# 🪖 Smart Helmet for Accident Detection & Emergency Alert System

## 📌 Overview
The **Smart Helmet for Accident Detection & Emergency Alert System** is an embedded IoT-based safety solution designed to automatically detect motorcycle accidents and instantly notify emergency contacts with the rider’s real-time location.  

The system is **fully autonomous**, does **not depend on smartphones**, and is engineered to operate even when the rider is unconscious—significantly reducing emergency response time during the critical *golden hour*.

---

## 🎯 Key Objectives
- Automatic accident detection using motion sensing
- Real-time GPS-based location tracking
- GSM-based emergency alert via SMS
- False-alarm prevention with user override
- Helmet-wear detection for system reliability
- Low-cost, portable, and scalable design

---

## ⚙️ System Architecture
**Core Components**
- **ESP32** – Main controller
- **MPU6050** – Accelerometer & Gyroscope (impact detection)
- **NEO-6M GPS** – Location tracking
- **SIM800L GSM** – SMS communication
- **TTP223 Touch Sensor** – Helmet wear detection
- **Buzzer + Push Button** – User interaction
- **Li-Po Battery + TP4056** – Power management

```

[ Helmet Worn ]
↓
[ MPU6050 Monitoring ]
↓
[ Crash Detected ]
↓
[ 15s Countdown + Buzzer ]
↓
[ GPS Location Acquired ]
↓
[ GSM SMS Alert Sent ]

```

---

## 🔄 Working Principle
1. System activates only when the helmet is worn.
2. MPU6050 continuously measures acceleration.
3. A crash is detected using **delta-G analysis with filtering**.
4. A 15-second countdown alerts the rider via buzzer.
5. Rider may cancel false alerts using a button.
6. If not cancelled:
   - GPS coordinates are acquired.
   - Emergency SMS with Google Maps link is sent.
7. System waits until helmet is removed to reset.

---

## 🧠 Accident Detection Logic
- Uses **RAW MPU6050 data** (no unstable library dependencies).
- Calculates acceleration magnitude.
- Maintains a dynamic baseline (~1g).
- Triggers when **delta-G exceeds threshold**.
- Low-pass filtering and hysteresis reduce false positives.

---

## 🧪 Testing Summary
| Test Scenario | Result |
|--------------|--------|
| Helmet not worn | System idle |
| Normal vibration | No alert |
| Sudden impact | Countdown triggered |
| Cancel button pressed | Alert cancelled |
| Countdown completed | SMS sent |
| Weak GSM signal | Automatic retry |

- **Detection Accuracy:** ~95%
- **Total Alert Time:** 15–20 seconds
- **SMS Delivery Reliability:** ~97%

---

## 🛠️ Technologies Used
### Hardware
- ESP32
- MPU6050
- SIM800L
- NEO-6M GPS
- TTP223 Touch Sensor
- Li-Po Battery + TP4056

### Software
- Arduino IDE
- Embedded C/C++
- I2C & UART Communication
- TinyGPS++, TinyGSM

---

## 🔌 ESP32 Pin Configuration
| Module | ESP32 GPIO |
|------|-----------|
| MPU6050 SDA | GPIO 21 |
| MPU6050 SCL | GPIO 22 |
| GPS TX / RX | GPIO 26 / 27 |
| GSM TX / RX | GPIO 17 / 18 |
| Helmet Sensor | GPIO 33 |
| Cancel Button | GPIO 32 |
| Buzzer | GPIO 25 |
| GSM Reset | GPIO 4 |

---

## ⚠️ Important Instructions
- Use a **2G-supported SIM card** (SIM800L).
- GSM module requires **high current (≈2A peak)**.
- Do **NOT** power GSM from ESP32 3.3V pin.
- Place GPS antenna near helmet ventilation for better lock.
- Calibrate crash threshold before deployment.

---

## 🔐 Security & Privacy Notice
- Emergency phone numbers are **masked with placeholders** in this repository.
- Replace `+91XXXXXXXXXX` with real numbers before flashing the firmware.
- No personal data is stored or transmitted beyond SMS alerts.

---

## 👨‍💻 Contributors
- Pawan Kumar Gupta    
- Nishant Kumar  
- Devansh Verma
- Himanshu Namboori

---
