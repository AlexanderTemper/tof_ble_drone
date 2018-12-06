# Spektrum RC with Betaflight using Simblee, ROS and TOF Sensor
Aim of this Project is to control a drone from the computer using a joystick. On Pc side ROS is used to read data from a joystick and send it over BLE to the Simblee. The Simblee then generates the Spektrum 2048 frame (only 7 channels are used) which is then sent over the UART to the flight controller. The flight controller sends back Telemetry using the LTM protocol. Also on the drone an Time of Flight sensor is present. This sensor communicates with the Simblee (over I2C) and sends it data also over BLE in an LTM Frame (custom one) to the PC.

Drone hardware used:

| Component  | |
| ------------- | ------------- |
| frame | RACE COPTER ALPHA 250Q   |
| flight controller | Matek F405 STD with Betaflight 3.2.2    |
| power distribution board | Matek FCHUB-6S |
| esc | Xrotor-30A-Micro-2-4S-BLHeli |
| motor | Xrotor-2205-2300KV |
| props | DAL 5045 V2 |
| battery | Tattu 75C Battery 1550mAh |
| Time of Flight Distance Sensor | Adafruit VL53L0X |
| Bluetooth Modul Drone | Simblee |


computer hardware used:

| Component  | |
| ------------- | ------------- |
| USB Bluetooth Dongle | LogiLink-BT0015 |
| Joystick | Sony DualShock 3 |

computer software used:

| Component  | |
| ------------- | ------------- |
| OS | Ubuntu-Mate 18.04 |
| ROS-Version | melodic,1.14.3 |

schematic:





# Setting up pygatt
```
sudo apt-get install python-pip
pip install pygatt
pip install pexpect
```

# Install and Building the ROS Nodes:

## Joy Node
```
sudo apt-get install ros-melodic-joy
sudo chmod a+rw /dev/input/jsX
```
replace X with your input device

## Controll Node

1. Create a new folder
```
mkdir -p tofWS
cd tofWS
```
2. Clone this repository 
```
git clone https://github.com/AlexanderTemper/tof_ble_drone.git
mv tof_ble_drone/ src
```
3. Create 
```
catkin_make
source devel/setup.bash
```

# Launching

```
roslaunch tof_ble_drone drone.launch --screen
``` 
you need too make adjustments in drone.launch to match your controller input device and your Bluetooth Device Address.

