import pandas as pd
import os
from collections import Counter
import sys

# Optional GUI imports
try:
    import customtkinter as ctk
    from tkinter import ttk, filedialog, messagebox
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False

# ============================================================
# Constants (matching config.h)
# ============================================================
LOW_ANGLE = 35
MAX_ANGLE = 75
CRITICAL_ANGLE = 25
MAX_GROUNDTIME_MS = 2000
MAX_CYCLETIME_MS = 3000
PARALLELISM_THRESH_DEG = 22.95 # Derived from qw > 0.2

# ============================================================
# Validation Logic (matching logic.ino)
# ============================================================
def validate_step(row):
    """
    Returns (Error Name, Color) if error found, else (None, None)
    """
    try:
        sa = float(row['StrikeAngle'])
        la = float(row['LiftAngle'])
        gms = float(row['GroundTime(ms)'])
        cms = float(row['CycleTime(ms)'])
        rot = float(row['StickRot(deg)'])
    except (ValueError, KeyError):
        return None, None

    lift_time = cms - gms

    # 1. lowPositionError - strike angle <= 35
    if sa <= LOW_ANGLE:
        return "Low Position", "#ff4d4d"
    
    # 2. rotateHipError - strike angle > 75 and liftTime < gms
    if sa > MAX_ANGLE and lift_time < gms:
        return "Rotate Hip", "#ffa500"
    
    # 3. elbowError - short ground time and angle > MAX
    if sa > MAX_ANGLE and lift_time >= gms:
        return "Elbow Error", "#ffff4d"

    # 4. motionRangeError and normal strike angle
    if sa < MAX_ANGLE and lift_time >= gms:
        return "Motion Range", "#4dffff"

    # 5. parallelOperationError - rotation threshold
    if rot > PARALLELISM_THRESH_DEG:
        return "Parallelism", "#ff4dff"

    # 6. pushError - very low lift angle or dragging
    if la < CRITICAL_ANGLE or gms >= MAX_GROUNDTIME_MS or cms >= MAX_CYCLETIME_MS:
        return "Push Error", "#4dff4d"

    return None, None

# ============================================================
# UI Components
# ============================================================

if GUI_AVAILABLE:
    class LogAnalyzerApp(ctk.CTk):
        def __init__(self):
            super().__init__()

            self.title("Nordic Walking Log Analyzer")
            self.geometry("1200x800")
            ctk.set_appearance_mode("dark")
            
            self.df = None
            self.file_path = None

            # Grid configuration
            self.grid_columnconfigure(0, weight=1)
            self.grid_rowconfigure(1, weight=1)

            # Header
            self.header_frame = ctk.CTkFrame(self, height=60)
            self.header_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=10)
            
            self.btn_open = ctk.CTkButton(self.header_frame, text="Open Log File", command=self.open_file)
            self.btn_open.pack(side="left", padx=10)
            
            self.lbl_status = ctk.CTkLabel(self.header_frame, text="No file loaded")
            self.lbl_status.pack(side="left", padx=10)

            # Main Content (Tabview)
            self.tabview = ctk.CTkTabview(self)
            self.tabview.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
            self.tabview.add("Log & Charts")
            self.tabview.add("Summary Statistics")
            
            # --- TAB 1: Log & Charts ---
            self.log_container = ctk.CTkFrame(self.tabview.tab("Log & Charts"))
            self.log_container.pack(fill="both", expand=True)
            self.log_container.grid_columnconfigure(0, weight=3) # Table
            self.log_container.grid_columnconfigure(1, weight=2) # Charts
            self.log_container.grid_rowconfigure(0, weight=1)

            # Table Frame
            self.table_frame = ctk.CTkFrame(self.log_container)
            self.table_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
            self.setup_table()

            # Chart Frame
            self.chart_frame = ctk.CTkFrame(self.log_container)
            self.chart_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
            self.lbl_chart = ctk.CTkLabel(self.chart_frame, text="Error Statistics Over Time", font=ctk.CTkFont(size=16, weight="bold"))
            self.lbl_chart.pack(pady=5)
            
            # Use a more modern style for the plot
            plt.style.use('dark_background')
            self.fig, self.ax = plt.subplots(figsize=(5, 4), dpi=100)
            self.fig.patch.set_facecolor('#1e1e1e')
            self.ax.set_facecolor('#1e1e1e')
            
            self.canvas = FigureCanvasTkAgg(self.fig, master=self.chart_frame)
            self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)

            # --- TAB 2: Summary ---
            self.setup_summary_tab()

        def setup_table(self):
            style = ttk.Style()
            style.theme_use("default")
            style.configure("Treeview", 
                            background="#2b2b2b", 
                            foreground="white", 
                            fieldbackground="#2b2b2b",
                            borderwidth=0,
                            font=('Arial', 10))
            style.configure("Treeview.Heading", background="#333333", foreground="white", relief="flat")
            style.map("Treeview", background=[('selected', '#1f538d')])

            self.tree = ttk.Treeview(self.table_frame, columns=("Step", "Time", "SA", "LA", "SF", "GMS", "CMS", "Rot", "Error"), show='headings')
            
            cols = {
                "Step": 50, "Time": 60, "SA": 50, "LA": 50, "SF": 60, "GMS": 70, "CMS": 70, "Rot": 60, "Error": 150
            }
            for col, width in cols.items():
                self.tree.heading(col, text=col)
                self.tree.column(col, width=width, anchor="center")

            self.tree.pack(side="left", fill="both", expand=True)
            
            scrollbar = ctk.CTkScrollbar(self.table_frame, orientation="vertical", command=self.tree.yview)
            scrollbar.pack(side="right", fill="y")
            self.tree.configure(yscrollcommand=scrollbar.set)

        def setup_summary_tab(self):
            self.summary_container = ctk.CTkScrollableFrame(self.tabview.tab("Summary Statistics"))
            self.summary_container.pack(fill="both", expand=True, padx=10, pady=10)
            
            # We will populate this dynamically
            self.summary_labels = {}

        def update_summary_display(self, stats):
            # Clear previous labels
            for widget in self.summary_container.winfo_children():
                widget.destroy()
            
            # Helper to create a section
            def add_section(title):
                lbl = ctk.CTkLabel(self.summary_container, text=title, font=ctk.CTkFont(size=18, weight="bold"), text_color="#1f538d")
                lbl.pack(anchor="w", pady=(15, 5))
                ctk.CTkFrame(self.summary_container, height=2, fg_color="#333333").pack(fill="x", pady=2)

            def add_row(label, value):
                frame = ctk.CTkFrame(self.summary_container, fg_color="transparent")
                frame.pack(fill="x", pady=1)
                ctk.CTkLabel(frame, text=label, width=200, anchor="w").pack(side="left")
                ctk.CTkLabel(frame, text=value, font=ctk.CTkFont(weight="bold")).pack(side="left", padx=20)

            # Totals
            add_section("Overall Training Data")
            add_row("Total Steps:", f"{stats['steps']}")
            add_row("Total Time:", f"{stats['duration']}")
            add_row("Technique Purity:", f"{stats['purity']:.1f}%")
            add_row("Total Errors:", f"{stats['total_errors']}")

            # Error Breakdown
            add_section("Technique Errors")
            for err, count in stats['techniqueErrors'].items():
                color = "white" if count == 0 else "#ff4d4d"
                frame = ctk.CTkFrame(self.summary_container, fg_color="transparent")
                frame.pack(fill="x", pady=1)
                ctk.CTkLabel(frame, text=f"{err}:", width=200, anchor="w").pack(side="left")
                ctk.CTkLabel(frame, text=f"{count}", font=ctk.CTkFont(weight="bold"), text_color=color).pack(side="left", padx=20)

            # Detailed Stats
            add_section("Detailed Metrics (Avg / Min / Max)")
            metrics_order = [
                ("Strike Angle (deg)", "strikeAngle"),
                ("Lift Angle (deg)", "liftAngle"),
                ("Range (deg)", "RangeAngle"),
                ("Strike Force (kgf)", "strikeForce"),
                ("Lift Force (kgf)", "liftForce"),
                ("Ground Time (ms)", "groundTime"),
                ("Cycle Time (ms)", "cycleTime"),
                ("Frequency (steps/min)", "frequency"),
                ("Rotation (deg)", "rotation")
            ]
            
            for label, key in metrics_order:
                m = stats[key]
                add_row(label, f"{m['avg']:.1f}  /  {m['min']:.1f}  /  {m['max']:.1f}")

        def calculate_stats(self):
            df = self.df
            
            # Helper for stats
            def get_mmm(col, decimals=1):
                return {
                    "avg": round(df[col].mean(), decimals),
                    "min": round(df[col].min(), decimals),
                    "max": round(df[col].max(), decimals)
                }

            # Calculate RangeAngle as Strike - Lift
            df['RangeCalc'] = df['StrikeAngle'] - df['LiftAngle']

            stats = {
                "steps": len(df),
                "duration": df.iloc[-1]['TimeLeft'] if 'TimeLeft' in df.columns else "00:00",
                "strikeAngle": get_mmm('StrikeAngle'),
                "liftAngle": get_mmm('LiftAngle'),
                "RangeAngle": get_mmm('RangeCalc'),
                "strikeForce": get_mmm('StrikeForce(kgf)', 2),
                "liftForce": get_mmm('LiftForce(kgf)', 2),
                "groundTime": get_mmm('GroundTime(ms)', 0),
                "cycleTime": get_mmm('CycleTime(ms)', 0),
                "frequency": get_mmm('Freq(s/m)', 1),
                "rotation": get_mmm('StickRot(deg)', 1),
            }

            # Technique Errors
            err_counts = Counter(df[df['DetectedError'] != ""]['DetectedError'])
            stats['techniqueErrors'] = {
                "Low Position": err_counts.get("Low Position", 0),
                "Rotate Hip": err_counts.get("Rotate Hip", 0),
                "Elbow Error": err_counts.get("Elbow Error", 0),
                "Motion Range": err_counts.get("Motion Range", 0),
                "Parallelism": err_counts.get("Parallelism", 0),
                "Push Error": err_counts.get("Push Error", 0),
            }
            stats['total_errors'] = sum(stats['techniqueErrors'].values())
            
            if stats['steps'] > 0:
                stats['purity'] = 100.0 - (stats['total_errors'] / stats['steps'] * 100.0)
            else:
                stats['purity'] = 100.0
                
            return stats

        def open_file(self):
            path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
            if path:
                self.load_data(path)

        def load_data(self, path):
            try:
                # Load CSV
                self.df = pd.read_csv(path)
                self.file_path = path
                self.lbl_status.configure(text=f"Loaded: {os.path.basename(path)} ({len(self.df)} steps)")
                
                # Process Errors
                self.process_errors()
                
                # Calculate Summary
                stats = self.calculate_stats()
                self.update_summary_display(stats)
                
                # Update UI
                self.populate_table()
                self.update_charts()
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load file:\n{str(e)}")

        def process_errors(self):
            errors = []
            error_colors = []
            for _, row in self.df.iterrows():
                err, color = validate_step(row)
                errors.append(err if err else "")
                error_colors.append(color)
            
            self.df['DetectedError'] = errors
            self.df['ErrorColor'] = error_colors

        def populate_table(self):
            # Clear table
            for item in self.tree.get_children():
                self.tree.delete(item)
            
            # Batch insertion for better performance
            for i, row in self.df.iterrows():
                tags = ()
                if row['DetectedError']:
                    tag_name = f"err_{i}"
                    self.tree.tag_configure(tag_name, foreground=row['ErrorColor'], font=('Arial', 10, 'bold'))
                    tags = (tag_name,)
                
                # Use 'TimeLeft' column for Time
                time_val = row['TimeLeft'] if 'TimeLeft' in row else "00:00"
                
                self.tree.insert("", "end", values=(
                    int(row['Step']), 
                    time_val,
                    f"{row['StrikeAngle']:.1f}", 
                    f"{row['LiftAngle']:.1f}", 
                    f"{row['StrikeForce(kgf)']:.2f}", 
                    int(row['GroundTime(ms)']), 
                    int(row['CycleTime(ms)']), 
                    f"{row['StickRot(deg)']:.1f}",
                    row['DetectedError']
                ), tags=tags)

        def update_charts(self):
            self.ax.clear()
            
            # Filter rows with errors
            err_df = self.df[self.df['DetectedError'] != ""]
            
            if len(err_df) == 0:
                self.ax.text(0.5, 0.5, "No errors detected", color='white', ha='center')
                self.canvas.draw()
                return

            # Frequency over time (using Step as x-axis)
            # Group by error type and plot counts in windows of 50 steps
            window_size = 50
            self.df['StepGroup'] = (self.df['Step'] // window_size) * window_size
            
            error_types = self.df[self.df['DetectedError'] != ""]['DetectedError'].unique()
            
            for err_type in error_types:
                subset = self.df[self.df['DetectedError'] == err_type]
                counts = subset.groupby('StepGroup').size()
                self.ax.plot(counts.index, counts.values, label=err_type, marker='o')

            self.ax.set_xlabel("Step Number")
            self.ax.set_ylabel("Error Count (per 50 steps)")
            self.ax.legend(facecolor='#2b2b2b', edgecolor='white', labelcolor='white')
            self.ax.grid(True, alpha=0.2)
            
            self.canvas.draw()

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        # Headless test mode
        path = sys.argv[2] if len(sys.argv) > 2 else "test_log.csv"
        print(f"Testing analyzer on {path}...")
        
        # Mocking necessary parts for calculation
        class MockApp:
            def __init__(self, path):
                self.df = pd.read_csv(path)
            
            def process_errors(self):
                errors = []
                for _, row in self.df.iterrows():
                    err, _ = validate_step(row)
                    errors.append(err if err else "")
                self.df['DetectedError'] = errors
            
            def calculate_stats(self):
                # Using the same logic as in LogAnalyzerApp (copy-pasted for test)
                df = self.df
                def get_mmm(col, decimals=1):
                    return {"avg": round(df[col].mean(), decimals), "min": round(df[col].min(), decimals), "max": round(df[col].max(), decimals)}
                df['RangeCalc'] = df['StrikeAngle'] - df['LiftAngle']
                err_counts = Counter(df[df['DetectedError'] != ""]['DetectedError'])
                stats = {
                    "steps": len(df),
                    "duration": df.iloc[-1]['TimeLeft'] if 'TimeLeft' in df.columns else "00:00",
                    "strikeAngle": get_mmm('StrikeAngle'),
                    "liftAngle": get_mmm('LiftAngle'),
                    "RangeAngle": get_mmm('RangeCalc'),
                    "techniqueErrors": err_counts,
                    "total_errors": sum(err_counts.values())
                }
                stats['purity'] = 100.0 - (stats['total_errors'] / stats['steps'] * 100.0) if stats['steps'] > 0 else 100.0
                return stats

        mock = MockApp(path)
        mock.process_errors()
        stats = mock.calculate_stats()
        
        print("\n--- Training Summary ---")
        print(f"Steps: {stats['steps']}")
        print(f"Duration: {stats['duration']}")
        print(f"Purity: {stats['purity']:.1f}%")
        print(f"Total Errors: {stats['total_errors']}")
        print("\nError Breakdown:")
        for err, count in stats['techniqueErrors'].items():
            print(f"  {err}: {count}")
        print("\nMetrics (Avg / Min / Max):")
        print(f"  Strike Angle: {stats['strikeAngle']['avg']} / {stats['strikeAngle']['min']} / {stats['strikeAngle']['max']}")
        print(f"  Lift Angle:   {stats['liftAngle']['avg']} / {stats['liftAngle']['min']} / {stats['liftAngle']['max']}")
        print(f"  Range:        {stats['RangeAngle']['avg']} / {stats['RangeAngle']['min']} / {stats['RangeAngle']['max']}")
    else:
        if GUI_AVAILABLE:
            app = LogAnalyzerApp()
            app.mainloop()
        else:
            print("Error: Tkinter or CustomTkinter not found. GUI mode unavailable.")
            print("To fix on macOS: brew install python-tk")
            sys.exit(1)
