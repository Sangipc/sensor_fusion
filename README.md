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

This project aims to facilitate the real-time tracking of knee joint angles using Movella DOT Xsens IMUs in a MacOS environment. The application addresses the gap in data acquisition protocols for MacOS and leverages machine learning algorithms for enhanced accuracy and ease of use. This tool is particularly useful for professionals in rehabilitation, sports science, and ergonomics.

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
