# Real-Time Knee Joint Angle Tracking Using Movella DOT Xsens IMUs on MacOS #

## Abstract ##

This project presents an approach for real-time knee joint angle tracking using Movella DOT Xsens Inertial Measurement Units (IMUs) within a MacOS environment. 
While established protocols exist for precise data retrieval in Windows and Linux environments, a deficiency persists in fundamental developmental efforts concerning data acquisition in MacOS. 
Accurate tracking of joint angles is crucial for understanding human movement patterns, assessing biomechanical function, and facilitating rehabilitation, sports science, and ergonomics. Traditional methods for tracking knee joint angles often rely on cumbersome and expensive methods, limiting their practicality. Leveraging inertial sensors with machine learning algorithms offers advantages in terms of affordability, ease of use, and mobility. This study focuses on the development and evaluation of a real-time machine learning-based approach for knee joint angle tracking using inertial sensors. The methodology involves interfacing with Movella DOT IMUs, data acquisition, processing, and utilizing Quaternion-based Inertial Motion Tracking Toolbox (qmt) for accurate angle estimation. The results illustrate a high level of accuracy in comparison to the true frequency, with minimal standard deviation. The proposed solution provides a reliable and accessible method for continuous monitoring of knee joint angles in various settings.

## Index Terms ##

* Sensors
* Machine Learning
* Knee Joint Angle
* Quaternion-based Filtering

## Table of Contents

1. [Introduction](#introduction)
2. [Features](#features)
3. [Requirements](#requirements)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Code Structure](#code-structure)

### Introduction

This project provides a real-time solution for tracking knee joint angles using Movella DOT Xsens Inertial Measurement Units (IMUs) within a MacOS environment. The system is designed to bridge the gap in data acquisition protocols specific to MacOS, addressing the lack of support and optimization for this platform compared to Windows and Linux.

#### What is this project about?

Knee joint angle tracking is essential for various applications including biomechanics research, rehabilitation, sports science, and ergonomics. Traditional methods for measuring joint angles often involve bulky equipment and high costs, which can limit their practical application in everyday settings.

This project leverages Movella DOT Xsens IMUs, which are compact, affordable, and highly accurate inertial sensors. The approach utilizes advanced quaternion-based filtering techniques to ensure accurate tracking even in dynamic and challenging conditions. 

The software is built to operate seamlessly on MacOS, using Python for its implementation. It manages real-time data acquisition from the IMUs, processes this data through multiple processes, and continuously computes the knee joint angles. This allows users to monitor joint angles effectively and obtain valuable insights into movement patterns and biomechanical function.

Overall, this project offers a reliable, cost-effective, and user-friendly solution for knee joint angle tracking, making it accessible for researchers, practitioners, and enthusiasts interested in biomechanics and human movement analysis.

### Features

* **Real-time data acquisition** from Movella DOT Xsens IMUs
* **Multi-process data handling** for efficient performance
* **Quaternion-based filtering** for accurate angle estimation
* **Cross-platform** compatibility (focused on MacOS)
* **Ease of use** with minimal setup and affordable equipment

### Requirements

* MacOS operating system
* Python 3.7 or higher
* Movella DOT Xsens IMUs
* Libraries: **asyncio**, **multiprocessing**, **bleak**, **atexit**, **numpy**, **qmt**, **VQF**, **matplotlib**

## Installation ##

1. Clone the repository:

```
git clone https://github.com/Sangipc/sensor_fusion.git
```

2. Ensure that all the required libraries are installed:

* `asyncio`
* `multiprocessing`
* `atexit`
* `bleak`
* `numpy`
* `qmt`
* `VQF`
* `matplotlib`

### Usage ###

1. **Run the main script:**

```

python main.py

```

2. **Process Overview:**

* The script will scan and filter Movella DOT Xsens IMUs.
* Data will be continuously read from the IMUs.
* The data will be processed in real-time to compute the knee joint angles.

### Code Structure ###

* **`main.py`**: Main entry point for the application.
* **`scanner.py`**: Scans and filters available Movella DOT Xsens IMUs.
* **`measurement.py`**: Handles the continuous reading of data from the IMUs.
* **`ble_sensor.py`**: Handles tasks such as connecting to sensors, configuring measurement modes, and recording sensor data.
It includes classes and functions to manage BLE communication, synchronize timestamps, convert sensor data, handle notifications, and log data into CSV files.
* **`dataprocessing.py`**: Contains the logic for processing the sensor data and visualizing using matplotlib.
