## Contents
1. [Introduction](#introduction)
2. [Mechanical Design and Manufacturing](#mechanical-design-and-manufacturing)
3. [Process Flow Diagram](#system-architecture)
4. [Installation](#installation)
5. [Usage](#usage)
7. [Results](#results)
8. [Contact](#contact)

## Introduction
In the study, the Create-3 robot, produced by IRobot for ROS2 developers and capable of differential drive, was used. An RFID system was installed on the robot, and the counting processes were carried out through this system.
<p align="center">
  <img src="images/IRobot_Create-3.jpg" alt="IRobot Create-3" />
</p>
<p align="center">IRobot Create-3</p>

## Mechanical Design and Manufacturing
The design of the study was carried out on Fusion 360. After the design, the production phase used aluminum profiles for the skeleton structure, and black plexiglass sheets were used for the intermediate layers.
<p align="center">
  <div style="display: flex; justify-content: center;">
    <img src="images/Design.jpg" style="margin-right: 20px;" />
    <img src="images/Physical_ System.jpg" />
  </div>
</p>
<p align="center">Design & Physical System</p>


## Process Flow Diagram
The Inventory Counting Robot must perform a series of tasks sequentially. There are tasks that need to be done in both serial and parallel. The first task for the robot is to establish communication between the Create-3 and the PC. To confirm that the communication is working properly, it is observed that the topics, services, actions, and parameters are coming through correctly on the PC. After these steps, the hardware connections for the lidar and RFID system are established. Once the hardware setup is complete, the software processes begin. First, the lidar driver package is activated. Since the legs of the lidar skeleton are perceived as obstacles, a laser filtering file is executed to filter that area. Next, the map obtained from the SLAM software is provided as a parameter, and the navigation software is activated, giving the robot its initial position. After setting the robot's starting position, the two software systems are started in parallel. The first one involves a pre-existing file with target points where the robot needs to go and return to the starting point. The second one is the publisher node that communicates with the inventory reader via serial communication and publishes the data it reads from the serial port for the bag record. Once the robot returns to the starting point, the navigation and counting tasks are completed.
<p align="center">
  <img src="images/Counting_Flow_ Diagram.jpg"/>
</p>
<p align="center">Process Flow Diagram</p>
## Installation
Kurulum adımları burada yer alacaktır.

## Usage
Projenin nasıl kullanılacağı anlatılacaktır.


## Results
Elde edilen sonuçlar burada paylaşılacaktır.

## Contact
İletişim bilgileri burada yer alacaktır.
