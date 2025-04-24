# Particle Filter Localization & Object Tracking

![Build Status](https://img.shields.io/badge/ROS-Foxy-blue)  
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

A set of ROS-based demos showcasing **particle filter** algorithms for state estimation and object tracking on both a robotic arm and a wheeled vehicle.  
Initially developed for Cornell’s CS4750 Robotic Foundations course. :contentReference[oaicite:0]{index=0}

---
## Overview

This project implements particle-filter-based **localization** and **object tracking** using the ROS ecosystem and Rviz for visualization :contentReference[oaicite:1]{index=1}.  
It processes visual data from an overhead camera to estimate the pose of a robotic arm and a ground vehicle in simulation.

---

## Features

- 🔧 **ROS Nodes** for particle filter prediction & update cycles  
- 📊 **Error Analysis** plots of positional accuracy  
- 📽️ **Rviz** visualization of particles and ground-truth trajectories  
- 🚗 & 🤖 Separate demos for a **car** and a **robotic arm** :contentReference[oaicite:2]{index=2}

---

## Demo

### Robotic Arm Tracking  
 
*Particle distribution converging on the object’s true pose.* :contentReference[oaicite:3]{index=3}

![pf_convergence](https://github.com/user-attachments/assets/e6b684b9-3206-4e93-a649-a49753fe3abd)


*Mean absolute error over time.* :contentReference[oaicite:4]{index=4}

![position_error_plot](https://github.com/user-attachments/assets/09b9888a-8e1a-455b-8133-d972c043415f)

### Car Localization  
<img src="images/car_localization.gif" alt="Car Localization Demo" width="600"/>  
*Particles localizing a simulated car in a 2D map.* :contentReference[oaicite:5]{index=5}

---

## Installation

1. **Clone the repo**  
   ```bash
   git clone https://github.com/tripa2020/state_estimation.git
   cd state_estimation
   ``` :contentReference[oaicite:6]{index=6}

2. **Install ROS 2 Foxy** (if not already)  
   ```bash
   sudo apt update && sudo apt install ros-foxy-desktop
   ``` :contentReference[oaicite:7]{index=7}

3. **Install dependencies**  
   ```bash
   pip3 install -r requirements.tx
