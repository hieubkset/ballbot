# Ballbot = Ball + Robot: Robot picking up the ball

![img](image/img2.jpg)

## Dependencies

The project is tested on Ubuntu 16.04, Python 2.7, and ROS Kinetic.

```
opencv-python==4.1.1.26
pyrealsense2==2.32.1.1299
```

## Calibration

It needs calibration before running. 

```
rosrun ballbot calibration.py
```

It first gets the pixel position of 4 markers on the table. You should define the color of markers on ```detector.py```. The default value is ```YELLOW```.

Then you need to move the endpoint of robot to 4 markers. It is to get the cartesian position of 4 markers.

# Demo

Run the following command:

```
roslaunch ballbot ballbot.launch
```

Video demo are available at [Youtube](https://www.youtube.com/watch?v=42_p_2_yxcc).



