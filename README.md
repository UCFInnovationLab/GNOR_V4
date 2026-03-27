# GNOR
![Orange](docs/orange.jpg)
![Hull](docs/boathull.gif)
## GNOR Description
The Great Navel Orange Race (GNOR) is an annual competition held at UCF every year for the second intro to engineering course. The project involves students building a boat, submarine, or other watercraft that autonomously carries an orange around the reflection pond.
The TI Innovation lab is providing students with TI microcontroller boards (MSP430f5529 LaunchPad, MSP-EXP432P401R LaunchPad) and an IMU (mpu6050) sensor for use in their watercrafts. These components empower students to control servos, provide signals for high power relays and ESCs, measure angle change relative to starting angle, look at accelerometer data, and more.
This repository provides everything needed to get started using these components. This includes example code, pinouts, and more. If you have any questions or are having trouble getting started, you can find help at the UCF Innovation Lab in ENGII room 112 9AM-10PM M-F and Saturday 10-5. 

---

## Supported Boards

This project supports the following three microcontroller boards:

| MSP-EXP430F5529LP | MSP-EXP432P401R | ESP32-CP2102 |
|:-----------------:|:---------------:|:------------:|
| ![MSP-EXP430F5529LP](docs/MSP-EXP430F5529LP.png) | ![MSP-EXP432P401R](docs/MSP-EXP432P401R.png) | ![ESP32-CP2102](docs/ESP32-CP2012.png) |
| MSP-EXP430F5529LP | MSP-EXP432P401R | ESP32-CP2102 |

---

## Arduino IDE 2.x Setup

This project requires additional board package URLs, standard Arduino libraries, and custom ZIP libraries. Follow the steps below to fully configure Arduino IDE 2.x.

### Step 1 - Open Arduino IDE Settings

1. Open the **Arduino IDE**.
2. On **macOS**, click **Arduino IDE** in the top menu bar, then select **Settings**.
3. On **Windows** or **Linux**, click **File**, then select **Preferences**.
4. The Arduino IDE settings window will open.

### Step 2 - Add the Additional Boards Manager URLs

1. In the settings window, locate the field labeled **Additional Boards Manager URLs**.
2. Copy and paste the following URLs into that field, with **one URL per line**:

       https://raw.githubusercontent.com/Andy4495/TI_Platform_Cores_For_Arduino/main/json/package_energia_optimized_index.json
       https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

3. If the field already contains other URLs, do not delete them. Add these new URLs on separate lines below the existing entries.
4. Click **OK**, or close the settings window to save your changes.

### Step 3 - Install the Required Board Packages

Install only the board package(s) that match the board(s) you are using.

#### MSP-EXP430F5529LP
1. In the Arduino IDE, click **Tools**.
2. Select **Board**, then click **Boards Manager**.
3. In the search box, type **MSP430**.
4. Locate the **Energia MSP430 boards** package by (Andy4495/Energia).
5. Click **Install** and wait for the installation to finish.
6. To select this board: **Tools → Board → Energia MSP430 Boards → MSP-EXP430F5529LP**

#### MSP-EXP432P401R
1. In the Arduino IDE, click **Tools**.
2. Select **Board**, then click **Boards Manager**.
3. In the search box, type **MSP432**.
4. Locate the **TI MSP432P4xx Launchpad board** package by (Andy4495/Energia).
5. Click **Install** and wait for the installation to finish.
6. To select this board: **Tools → Board → Energia MSP432 EMT RED Boards → MSP-EXP432P401R**

#### ESP32-CP2102
1. In the Arduino IDE, click **Tools**.
2. Select **Board**, then click **Boards Manager**.
3. In the search box, type **esp32**.
4. Locate **esp32 by Espressif Systems**.
5. Click **Install** and wait for the installation to finish.
6. To select this board: **Tools → Board → esp32 → ESP32 Dev Module**

### Step 4 - Install the Required Libraries from Library Manager

1. In the Arduino IDE, click **Tools**.
2. Select **Manage Libraries...**
3. In the **Library Manager** search box, type **ESP32Servo**.
4. Locate **ESP32Servo** in the results list.
5. Click **Install** and wait for the installation to complete.
6. In the search box, type **NeoPixelBus**.
7. Locate **NeoPixelBus by Makuna**.
8. Click **Install** and wait for the installation to complete.
9. Confirm that both libraries are installed before continuing.

### Step 5 - Install Custom Libraries from ZIP Files

This project also requires the following custom libraries:

- [WS2812_MSP432](https://github.com/UCFInnovationLab/WS2812_MSP432/releases)
- [WS2812_MSP430](https://github.com/UCFInnovationLab/WS2812_MSP430/releases)
- [MPU6050](https://github.com/ucfinnovationlab/mpu6050/releases)

For each custom library, complete the following steps:

1. Open the library’s **Releases** page in your web browser.
2. Download the ZIP file for the latest recommended release.
3. Return to the Arduino IDE.
4. Click **Sketch**.
5. Select **Include Library**.
6. Click **Add .ZIP Library...**
7. Browse to the ZIP file you downloaded.
8. Select the ZIP file.
9. Click **Open** to install the library.
10. Repeat this process for each remaining custom library.

Using ZIP files from the **Releases** page is recommended because they provide a specific tested version of the library rather than the latest development snapshot from the repository.

### Step 6 - Restart the Arduino IDE and Verify Installation

1. Close the Arduino IDE.
2. Reopen the Arduino IDE.
3. Click **Tools → Board** and verify that the newly installed board packages are now available.
4. Click **Sketch → Include Library** and verify that the installed libraries appear in the library list.
5. Click **File → Examples** and check whether example sketches are available for the installed libraries.
6. Select your target board from the **Tools → Board** menu.
7. Open your project sketch.
8. Run a compile test to confirm that the required boards and libraries were installed correctly.

### Notes

- If the **Additional Boards Manager URLs** field already contains entries, keep them and add these new URLs on separate lines.
- Do not remove existing URLs unless you are sure they are no longer needed.
- If you previously installed older versions of the custom ZIP libraries manually, remove the older copies first to avoid duplicate library conflicts.
- After installation, example sketches may appear under **File → Examples**.

---

## Software Download

Before opening the project in the Arduino IDE, you need to download the GNOR_V4 sketch from GitHub. There are two ways to do this:

### Option 1 — GitHub Desktop (Recommended)

GitHub Desktop is a free application that lets you clone and manage repositories with a graphical interface.

**Advantages:**
- If a bug fix or update is released, you can pull the latest changes with a single click — no need to re-download and re-extract a ZIP file.
- Your local copy stays linked to the repository, making it easy to stay up to date throughout the competition.

**Steps:**
1. Download and install [GitHub Desktop](https://desktop.github.com) if you have not already.
2. Open GitHub Desktop.
3. Click **File → Clone Repository**.
4. Select the **URL** tab.
5. Enter the repository URL: `http://github.com/ucfinnovationlab/GNOR_V4`
6. Choose a local folder to save the project.
7. Click **Clone**.
8. Once cloned, open the `GNOR_V4.ino` file in the Arduino IDE.

To update the project later, open GitHub Desktop and click **Fetch origin**, then **Pull** to download any new changes.

### Option 2 — Download ZIP

**Advantages:**
- No additional software required — works with just a web browser and file explorer.
- Quick and simple for a one-time download.

**Steps:**
1. Open `http://github.com/ucfinnovationlab/GNOR_V4` in your web browser.
2. Click the green **Code** button near the top right of the page.
3. Select **Download ZIP**.
4. Save the ZIP file to your computer.
5. Extract the ZIP file to a folder of your choice. Ensure the folder is named `GNOR_V4`.
6. Open the extracted `GNOR_V4/GNOR_V4.ino` file in the Arduino IDE.

Note: if updates or bug fixes are released, you will need to repeat this process and re-download the ZIP to get the latest version.

---

## GNOR V4 Green Board Wiring

![GNOR PCB V4](docs/GNOR_pcb_v4.png)

### Single Rudder Boat Wiring

#### Rudder Servo
Connect the rudder servo to the **Servo1** connector on the board. Mind the orientation: the **black wire must align with the white line** on the board.

> ⚠️ **WARNING: Incorrect orientation will damage the board and/or servo. Double-check the wire orientation before applying power.**

#### ESC (Electronic Speed Controller)
Connect the ESC signal cable to the **ESC** connector on the board. Mind the orientation: the **black wire must align with the white line** on the board.

> ⚠️ **WARNING: Incorrect orientation will damage the board and/or ESC. Double-check the wire orientation before applying power.**

#### Motor Switch
Connect the motor switch to the **Motor** connector on the board.

### Dual Motor Boat Wiring

#### Left ESC
Connect the left ESC signal cable to the **Servo2** connector on the board. Mind the orientation: the **black wire must align with the white line** on the board.

> ⚠️ **WARNING: Incorrect orientation will damage the board and/or ESC. Double-check the wire orientation before applying power.**

#### Right ESC
Connect the right ESC signal cable to the **ESC** connector on the board. Mind the orientation: the **black wire must align with the white line** on the board.

> ⚠️ **WARNING: Incorrect orientation will damage the board and/or ESC. Double-check the wire orientation before applying power.**

#### Motor Switch
Connect the motor switch to the **Motor** connector on the board, same as the single rudder configuration above.

### ESC Calibration

ESC calibration teaches the ESC the zero throttle and maximum throttle positions so that it can use the full throttle range. This only needs to be done once, or any time the ESC is replaced.

**Steps:**
1. Ensure the board is powered off.
2. Short the two **Calibration** pins on the green board (e.g. using a jumper or a piece of wire).
3. Power up the board.
4. Wait until you hear the special calibration beeps from the ESC — this signals that the maximum throttle position has been registered.
5. Remove the short from the Calibration pins.
6. You should hear another beep from the ESC confirming that calibration is complete and the zero throttle position has been registered.
7. The ESC is now calibrated and ready for use.
