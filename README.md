# Tortabot

## How to launch torta to accept cmd_vel

### Software
execute on pi

This will source needed scripts
```
torta_build
```

launch low level scripts
```
roslaunch torta-bringup robot_low_level_w_odom.launch
```

publish velocity 
```
rosrun torta_web_control vel_pub.py x y yaw
```

### Hardware
- connect pi to supply

- connect pi usb to stm usb

- connect batteries to power jack


### Arm setup (for robotics)

- start rosserial

- publish arm values

- connect arm power


