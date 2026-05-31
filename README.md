# Voice-Controlled Color-Following Robot (PiCar-4WD)

An interactive, voice-controlled robot system built on top of the **PiCar-4WD** platform. This project splits the workload efficiently: a laptop handles heavy lifting like Automatic Speech Recognition (ASR) via OpenAI's Whisper, and streams lightweight UDP commands to a Raspberry Pi for real-time OpenCV color tracking and motor control.

---

## 🚀 Key Features

* **Push-to-Talk Voice Control**: Hold `SPACE` on your laptop to issue English voice commands, and release to transmit.
* **OpenCV Object Tracking**: Real-time HSV-based color masking and tracking for `red`, `blue`, `green`, `yellow`, and `pink` objects.
* **Smart Switch & Search Behavior**: When switching target colors, the robot pauses for 3 seconds to look for the new object. If not found, it enters a 360° scan mode rotating in place to search.
* **Time-Calibrated Manual Movements**: Supports precision manual overrides (e.g., "turn left 90 degrees and go straight 30 cm") using fine-tuned time-based calibration.

---

## 📂 System Architecture & File Structure

* `nlp.py` *(Runs on Laptop)*: Captures microphone audio, transcribes it using Whisper (`tiny` model), parses natural language into UDP command strings, and sends them to the Pi.
* `robot_color_follow.py` *(Runs on Raspberry Pi)*: The main runtime script. It captures camera frames, tracks the target color contour using OpenCV, and drives the PiCar wheels.
* `robot_udp_receiver.py` *(Runs on Raspberry Pi)*: A non-blocking UDP socket server that polls for incoming commands from the laptop.

---

## 📋 Supported Voice Commands

### 1. Object Following Mode
* *"Follow red"* / *"Track the green object"* / *"Switch to yellow"*
* The robot will approach the object and maintain a predefined distance zone (`DESIRED_AREA`).

### 2. Manual Movement Commands
* `go forward N cm` / `go straight N cm`
* `turn left N degree` / `turn right N degree` / `turn clockwise N degree`
* **Sequences**: Commands can be chained using "and" (e.g., *"turn left 90 degrees and go straight 30 cm"*).

### 3. Emergency Brake
* *"Stop"* / *"Pause"* / *"Halt"* (Instantly resets manual queues and search states).

---

## ⚙️ Setup & Installation

### 1. Install Dependencies
Ensure you have Python 3.8+ installed on both devices. Clone this repository on both your laptop and Raspberry Pi, then run:

```bash
pip install -r requirements.txt
```
**Note for Laptop** : pyaudio requires system-level portaudio libraries. If the installation fails, install it first via your package manager:

macOS: brew install portaudio

Ubuntu/Linux: sudo apt install portaudio19-dev

**Note for Raspberry Pi**: Make sure your PiCar-4WD official library and picamera2 environment are already activated and working before running the project.

### 2.Configuration before Launch
1）Open nlp.py and modify the RPI_IP constant to match your Raspberry Pi's actual IP address on the local network:

```python
RPI_IP = "192.168.1.100"  # Replace with your Raspberry Pi's IP address
```
2）(Optional) If the robot overshoots distances or angles during manual commands, tune these calibration constants in robot_color_follow.py:
```python
SECONDS_PER_CM = 0.040
SECONDS_PER_DEGREE = 0.012
```

### 3.Running the Project
1）Start the Robot: On the Raspberry Pi, execute:
```bash
python robot_color_follow.py
```
2）Start the Voice Client: On your Laptop, execute:
```bash
python3 nlp.py
```
3）Hold the Spacebar to talk, release to send the command. Press ESC to exit the laptop client.




