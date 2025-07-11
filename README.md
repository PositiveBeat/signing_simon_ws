# Signing Simon Workspace

ROS 2 workspace for the robot project "Signing Simon", a robot hand on a robot arm. 

The [hand](https://red-rabbit-robotics.ghost.io/rx1-humanoid-hand-assembly-instruction/) and [arm](https://www.instructables.com/EEZYbotARM-Mk2-3D-Printed-Robot/) are seperate open source projects. 

## Preparing the Pi

Tested with a Raspberry Pi 4B flashed with Ubuntu 22.04 Server. 

Add your user to the needed groups (dialout, GPIO, I2C):
```
sudo usermod -aG dialout,gpio,i2c $USER
```

Install RPi.GPIO:
```
sudo apt install python3-rpi.gpio
```


### Setup I2C

Ubuntu Server won't have I2C enabled by default like Raspberry Pi OS.
Run:
```
sudo apt install -y raspi-config
sudo raspi-config
```
In the menu:

- Go to Interface Options
- Enable I2C

Then confirm I2C is available:
```
ls /dev/i2c-1
```
If that file exists, I2C is ready.