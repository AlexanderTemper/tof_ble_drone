# Project Spektrum RC with Betaflight and Simblee

Aim of this Project is to control a drone from the computer using a joystick. On Pc side Ros is used to read data from a joystick and send it over BLE to the simblee. The Simblee then generates the Spektrum 2048 frame (only 7 channels are used) which is then sent over the UART to the flight controller.

Components used:

| Component  | |
| ------------- | ------------- |
| Flightcontroller | Matek F405 STD with Betaflight 3.2.2   |
| Bluetooth Modul Drone | Simblee |
| USB Bluetooth Dongle | LogiLink-BT0015 |
| OS | Ubuntu-Mate 18.04 |
| ROS-Version | melodic,1.14.3 |
| Joystick | Sony DualShock 3 |


## Setting up pygatt
```
sudo apt-get install python-pip
pip install pygatt
pip install pexpect
```
## Install and Building the ROS Nodes:

....


