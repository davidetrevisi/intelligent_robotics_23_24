# GROUP 28 - BIFFIS NICOLA, PAVAN STEFANO, TREVISI DAVIDE

## Exercise 4
### Exercise 4.3
To know in advance the size of the vector of data coming from the laser, it is sufficient to do: vector_size = angle_max / angle_increment = 6.28 / 0.0087 = 720.

Conversion of the polar coordinates to cartesian (x,y) coordinates: x = r * cos(θ); y = r * sin(θ);

Each readout of the laser range finder converted to [x, y] coordinates is stored in the leg_ranges vector;

---

NOTE: We made some assumptions for our solution: first, that each leg must be separated by at least one 'infinite' value of the laser scanner (e.g. no walls behind the people for at least 3.5m), then each leg is placed according to the given standard (e.g. each person stands in a circle around the laser and there are no overlapping legs or people turning their sides to the robot so that only one leg is actually detected).

It is possible to do this with the proposed "general solution" described in Exercise 4.4 paragraph, but we decided to keep both methods instead.

---

To compute the position of each person we used a standard mathematical approach which consists in a "for" cycle iterating on each range found by the laser scan, if the range is not infinite (!std::isinf(ranges[i])) it is converted into (x,y) cartesian coordinates and then stored into a vector (leg_ranges), if the next range is infinite the program recognizes that the points for this "leg" are finished and starts to store the point for the next leg in another vector until it reaches the end of the ranges (720). Then it computes the average of the points for each leg and the average for each person, finding 3 positions:
- Person 1 Position: (x, y) = (1.934557, -0.008540);
- Person 2 Position: (x, y) = (1.071924, 2.020443);
- Person 3 Position: (x, y) = (1.036030, -1.998724).


### Exercise 4.4
To compute the position of any number of people, i.e. removing the assumption that there are three people, we used the kmeans algorithm from the OpenCV library and chose leg_count / 2 (computed with the previous algorithm) as k.

Position of each person: 
- (x, y) = (1.9355006, -0.0082151974);
- (x, y) = (1.0901387, 2.0032334);
- (x, y) = (1.0504487, -1.9953566).


To run the solution, execute the following commands in 3 different terminal windows:

1
```
cd catkin_ws 
catkin build
roscore
```
2
```
cd catkin_ws/src/exercise_4_G28/src
rosbag play -l bag_es_5.bag
```
3
```
cd catkin_ws
rosrun exercise_4 exercise_4
```

OpenCV 4 is required, theoretically it should already be installed along with ROS.
