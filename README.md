# Let's Just Dance

## Team Members: Leandra Nealer, Joyce Passananti, Daria Shifrina


<p align="center">
  <img src="mapgrid.png" alt="City Grid"/>
</p>

Final Project Repository for Introduction to Robotics. 



## Running Instructions 

### Update your source first:

```
$ cd ~/catkin_ws && catkin_make
$ source devel/setup.bash  
```


### Terminals:

```
$ roscore
```

```
$ roslaunch lets_just_dance turtlebot3_custom_world.launch
```
```
$ rosrun lets_just_dance robotmovement.py
```

```
$ rosrun lets_just_dance navigationui.py
```

### Navigation UI:

<p align="center">
  <img src="navigationui.png" alt="Navigation UI"/>
</p>

### References:

1. Multiple turtles: https://github.com/francimala/ROS_multiple_navigation
