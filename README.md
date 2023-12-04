# Intelligent Robotics 2023-2024

Repository containing the solutions to the Intelligent Robotics exercises

## Exercise 4

The solution for this exercise is divided in two parts:
- for the assumption that the number of people is known, we used a standard mathematical approach ... .
- if the number of people is unknown, we used **kmeans** algorithm from the OpenCV library and selected *leg_count / 2* as **k**.

To run the solution, execute the following commands in 3 different terminal windows:
```
roscore
rosbag play -l bag_es_5.bag
rosrun exercise_4 exercise_4
```

OpenCV 4 is required, in theory it should have been already installed along ROS.
