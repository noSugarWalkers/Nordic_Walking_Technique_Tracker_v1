import random

header = "Step,StrikeAngle,LiftAngle,StrikeForce(kgf),LiftForce(kgf),AccHoriz(g),GroundTime(ms),CycleTime(ms),Freq(s/m),VibeDuration,VibeFreq,StickRot(deg),TimeLeft"

with open("test_log.csv", "w") as f:
    f.write(header + "\n")
    for i in range(1, 501):
        # Default normal values
        sa = 45 + random.uniform(-5, 5)
        la = 40 + random.uniform(-5, 5)
        sf = 0.5 + random.uniform(-0.1, 0.1)
        lf = 0.3 + random.uniform(-0.1, 0.1)
        ah = 0.2 + random.uniform(-0.05, 0.05)
        gt = 1000 + random.randint(-50, 50)
        ct = 1800 + random.randint(-100, 100)
        fr = 60000.0 / ct
        vd = 20 + random.uniform(-5, 5)
        vf = 100 + random.uniform(-10, 10)
        rot = 10 + random.uniform(-5, 5)
        tl = "10:00"

        # Inject errors periodically
        if i % 50 == 10: sa = 30 # Low Position
        elif i % 50 == 20: # Rotate Hip
            sa = 80
            gt = 1000
            ct = 1500
        elif i % 50 == 30: # Elbow Error
            sa = 80
            gt = 500
            ct = 1500
        elif i % 50 == 40: # Motion Range
            sa = 60
            gt = 500
            ct = 1500
        elif i % 50 == 5: rot = 30 # Parallelism
        elif i % 50 == 15: la = 10 # Push Error (Low Angle)
        elif i % 50 == 25: gt = 2000 # Push Error (Dragging)

        line = f"{i},{sa:.1f},{la:.1f},{sf:.2f},{lf:.2f},{ah:.2f},{int(gt)},{int(ct)},{fr:.1f},{vd:.1f},{vf:.1f},{rot:.1f},{tl}"
        f.write(line + "\n")
print("test_log.csv created")
