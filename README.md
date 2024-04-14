# LidarFeatureExtraction


## Overview
This repository contains code for a robot with two main features: autonomous navigation and feature detection. The robot is designed to navigate its environment while avoiding obstacles using a combination of sensors and algorithms. Additionally, it is equipped with a feature detection system that creates a point cloud from LiDAR data and detects features in the surroundings.

# Example - Robot mapping a room
![roomMapExample](https://github.com/KarthiU/LidarFeatureExtraction/assets/112822491/5a9c0e3a-d117-4529-9865-27b2659066fe)

## Features
### Robot Navigation
- The robot utilizes algorithms to navigate autonomously.
- It is capable of avoiding obstacles using sensor data.

### Feature Detection System
- Utilizes LiDAR data to create a point cloud representation of the environment.
- Implements feature detection algorithms to identify key features in the surroundings.

## Dependencies
- ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
- [![NumPy](https://img.shields.io/badge/NumPy-blue?style=for-the-badge&logo=numpy)](https://numpy.org/)
- [![SciPy](https://img.shields.io/badge/SciPy-white?style=for-the-badge&logo=scipy)](https://numpy.org/)


## References 
Feature detection algorithm based on proposed seed region growing found in this [paper](https://journals.sagepub.com/doi/pdf/10.1177/1729881418755245)
