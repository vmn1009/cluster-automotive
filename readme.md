# 🚗 Cluster Automotive  

## 📌 Introduction  
The **Cluster Automotive** project uses **Raspberry Pi 4** combined with **STM32** to read velocity and position data throught UART driver, calculate motor speed, and apply PID control for optimized motor performance.  

## ⚙️ Key Features  
- 📡 **Read encoder data** on STM32 to measure motor speed.  
- 🔄 **Data communication** between Raspberry Pi 4 and STM32 throught UART driver.  
- ⚙️ **PID control** for precise and stable motor operation.  

## 🛠️ Hardware Used  
- **Raspberry Pi 4** – Handles data processing and communicates with STM32.  
- **STM32F401** – Reads encoder signals and computes speed.  
- **Encoder** – Measures the rotational speed of the motor.  

## 📂 Directory Structure  
```plaintext
📁 Cluster-Automotive  
 ├── 📁 QT/                     # UI application for Raspberry Pi  
 ├── 📁 control_dc_motor/          # STM32 code for motor control  
 ├── 📁 scripts/                # Utility scripts  
 ├── 📄 README.md               # Project documentation  
 └── 🎥 video1.mp4, video2.mp4  # Demonstration videos  
```

## 🎥 Demonstration  
**This video show the source code and project testing**
![Motor Running](video/video1.gif)

