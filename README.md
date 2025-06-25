# **Music Bike: Adaptive Music System for Cyclists**
### **Overview**
Music Bike is an embedded systems project aimed at creating a hardware-software solution that adapts music playback in real time by leveraging sensors to collect data like speed, pitch, roll, yaw, angular velocity, and acceleration. With these parameters, Music Bike can adapt the tempo, instrumentation, and endless other musical elements powered by the professional audio engine FMOD Studio.

#### Project Demo

[![Music Bike Project Demo](https://img.youtube.com/vi/FAYNOim8kmM/maxresdefault.jpg)](https://youtu.be/FAYNOim8kmM)

#### **Media Coverage**
- **[Hackaday Feature](https://hackaday.com/2025/06/22/an-adaptive-soundntrack-for-bike-tricks)** - "An Adaptive Soundtrack for Bike Tricks" - June 2025

---

### **Features**
- **Real-Time Music Adaptation**: Adjusts music playback based on riding speed, pitch, jumps, 180s, and other motion data.  
- **Sensor Integration**: Utilizes multiple sensors (e.g., speed sensor, accelerometer, gyroscope) to collect live data.  
- **Wireless Connectivity**: Transmits sensor data to a mobile app via Bluetooth Low Energy (BLE).  
- **App Integration**: Uses FMOD Studio and BLE to play adaptive music via an Android app.

---

### **Project Goals**
1. Develop hardware capable of accurately collecting bicycle data using sensors.  
2. Design algorithms to map sensor data to dynamic music parameters.  
3. Build a mobile app for real-time interaction and customization.  
4. Test the system in controlled and real-world cycling environments.

---

### **Technologies Used**
- **Microcontroller**: ESP32S3 for wireless connectivity and sensor integration.  
- **Sensors**: 2 Speed sensors (hall effect) placed on rear wheel, IMU (MPU9250) placed on central pillar
- **Programming Languages**: C++, C#, C for FMOD API integration, Kotlin for Android app, C++/Arduino for embedded code
- **Mobile App**: The bridge connecting project pieces (FMOD, embedded system). Includes developer tools like machine learning data collection, raw sensor output. 

---

### **Repository Structure**
```
├── /FMODSetup          # Code and examples for FMOD API
├── /MusicBike          # Android app used to connect to embedded system and play music
├── /embedded           # Schematics and sensor integration code for bike hardware
├── /tensorflow         # Experimental training data and scripts for trick detection machine learning, currently unused
└── README.md           # Project overview
```

---

### **Team Members**
- *Bennett Lahn* – Computer engineering student @ University of Washington
- *Nick Baroody* – Electrical & computer engineering 2025 alumni, University of Washington
- *Ethan Diep* – Electrical & computer engineering 2025 alumni, University of Washington
- *Matthew Pham* - Electrical & computer engineering 2025 alumni, University of Washington

---

### **Contributing**
Anyone interested in contributing is encouraged to fork the project and submit a pull request. We can't wait to see what you'll do with our platform!

---

### **License**
See MIT License. Please note that FMOD Studio and the FMOD Studio API have their own licensing requirements that should be respected. FMOD Studio licensing can be reviewed [here](https://www.fmod.com/licensing). All sound effects used in the app or FMOD bank files within this respository are in the public domain.

---

### **Contact**
Please create an issue or pull request. We're happy to help!
