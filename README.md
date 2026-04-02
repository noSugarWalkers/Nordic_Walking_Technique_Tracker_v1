<h1 align = "center">🌟Nordic Walking Technique Tracker🌟</h1> 

## 1️⃣Product
A compact, ESP32‑S3–based device designed to analyze and improve Nordic walking technique using advanced motion sensor BH260AP.

**🚶‍♂️ What It Does**
The tracker measures key parameters of Nordic walking technique in real time, capturing precise motion data to help athletes, trainers, and enthusiasts better understand their movement patterns. With a built‑in intelligent sensor, the device can detect gait phases, pole‑plant timing, asymmetries, posture deviations, and other biomechanical characteristics.


**🔧 Hardware Overview(LILYGO T-Display Bar without Display)**

ESP32‑S3 — the main controller providing wireless connectivity, low‑power operation.

Bosch BHI260AP — a 6‑axis smart IMU with integrated AI capabilities, delivering highly accurate motion tracking and gesture recognition.

BQ27220 - Battery level monitor meter

BQ25896 - Power control chip routines

SD Card controler

Buzzer

Together, these components enable high‑quality motion monitoring and logging while keeping the device small, energy‑efficient, and ready for field use.

## 2️⃣PinMap
| GPIO | SPI      | I2C     |  BUZZER     | BHI260AP(I2C) | BUTTON  | SD(SPI) | USB   | BQ27220 | FLASH128M | UART  |
| ---- | -------- | ------- |  ---------- | ------------- | ------- | ------- | ----- | ------- | --------- | ----- |
| IO2  |          | I2C_SDA |             |               |         |         |       |         |           |       |
| IO3  |          | I2C_SDA |             |               |         |         |       |         |           |       |
| IO12 | SPI_MOSI |         |             |               |         |         |       |         |           |       |
| IO13 | SPI_MISO |         |             |               |         |         |       |         |           |       |
| IO14 | SPI_SCK  |         |             |               |         |         |       |         |           |       |
| IO6  |          |         |             |               |         |         |       |         |           |       |
| IO7  |          |         |             |               |         |         |       |         |           |       |
| IO8  |          |         |             |               |         |         |       |         |           |       |
| IO5  |          |         |             |               |         |         |       |         |           |       |
| IO40 |          |         |             |               |         |         |       |         |           |       |
| IO15 |          |         |             |               |         |         |       |         |           |       |
| IO18 |          |         |             | BHI260AP_IRQ  |         |         |       |         |           |       |
| IO17 |          |         |             | BHI260AP_RST  |         |         |       |         |           |       |
| IO16 |          |         |             | BHI260AP_EN   |         |         |       |         |           |       |
| IO11 |          |         |             |               |         | SD_CS   |       |         |           |       |
| IO9  |          |         |  BUZZER_PIN |               |         |         |       |         |           |       |
| IO21 |          |         |             |               |         |         |       |         |           |       |
| IO1  |          |         |             |               |         |         |       |         |           |       |
| IO38 |          |         |             |               | Button1 |         |       |         |           |       |
| IO0  |          |         |             |               | Button2 |         |       |         |           |       |
| IO19 |          |         |             |               |         |         | USB_N |         |           |       |
| IO20 |          |         |             |               |         |         | USB_P |         |           |       |
| IO10 |          |         |             |               |         |         |       | GPOUT   |           |       |
| IO27 |          |         |             |               |         |         |       |         | SPIHD     |       |
| IO28 |          |         |             |               |         |         |       |         | SPIWP     |       |
| IO29 |          |         |             |               |         |         |       |         | SPICS0    |       |
| IO30 |          |         |             |               |         |         |       |         | SPICLK    |       |
| IO31 |          |         |             |               |         |         |       |         | SPIQ      |       |
| IO32 |          |         |             |               |         |         |       |         | SPID      |       |
| IO44 |          |         |             |               |         |         |       |         |           | U0RXD |
| IO43 |          |         |             |               |         |         |       |         |           | U0TXD |

## 3️⃣ Arduino IDE Quick Start

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. In Arduino Preferences, on the Settings tab, enter the `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` URL in the `Additional boards manager URLs` input box. **Please pay attention to the version. The test phase is using 2.0.14. It is not certain that versions above 2.0.14 can run. When the operation is abnormal, please downgrade to a version below 2.0.14.** , As of 2024/08/02, TFT_eSPI does not work on versions higher than 2.0.14, see [TFT_eSPI/issue3329](https://github.com/Bodmer/TFT_eSPI/issues/3329)
3. Open ArduinoIDE ,`Tools` ,Make your selection according to the table below

    | Arduino IDE Setting                  | Value                                   |
    | ------------------------------------ | --------------------------------------- |
    | Board                                | **ESP32S3 Dev Module**                  |
    | Port                                 | Your port                               |
    | USB CDC On Boot                      | Enable                                  |
    | CPU Frequency                        | 240MHZ(WiFi)                            |
    | Core Debug Level                     | None                                    |
    | USB DFU On Boot                      | Disable                                 |
    | Erase All Flash Before Sketch Upload | Disable                                 |
    | Events Run On                        | Core1                                   |
    | Flash Mode                           | QIO 80MHZ                               |
    | Flash Size                           | **16MB(128Mb)**                         |
    | Arduino Runs On                      | Core1                                   |
    | USB Firmware MSC On Boot             | Disable                                 |
    | Partition Scheme                     | **8M with spiffs(3M APP/1.5MB SPIFFS)** |
    | PSRAM                                | **OPI PSRAM**                           |
    | Upload Mode                          | **UART0/Hardware CDC**                  |
    | Upload Speed                         | 921600                                  |
    | USB Mode                             | **CDC and JTAG**                        |

    * The options in bold are required, others are selected according to actual conditions.
4. Download and install SensorLib, XPowersLib, SdFat libs. Other peripherals lib [Xinyuan-LilyGO/T-Display-Bar](https://github.com/Xinyuan-LilyGO/T-Display-Bar/tree/master/lib) if needed.
4. Click `upload` , Wait for compilation and writing to complete
5.  If it cannot be written, or the USB device keeps flashing, please check the **FAQ** below
