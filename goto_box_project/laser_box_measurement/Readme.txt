This package is used on ROS indigo igloo Ubuntu 14.04 LTS

This package provides the measurement of the pose ground truth of the wheelchair robot.
It uses a laser scanner to measure the distance between a reference and pole object on the wheelchair. 
We used very high reflective material on this specific pole to make it visible at all different spots. 
The material on this pole is quite small in diameter (around 3.5 to 4cm) 
Make sure that the cardboard box around the pole is high enough that the laser is able to see this as 1 object only.
The laser of the white head sensor box is used to measure the distance between the pole and the reference object

This package provides odometry between ref. and pole position
This package provides tf transform between ref. and pole position.

first you need to know where you put your reference measurement. For that, this package provides the measurement_laser_position.launch.
The basic idea is that amcl is converting to the laser position that you're currently using for a reference measurement. For that, you have to run a hokuyo_node on the
white laser (sensor) box (shell into the odroid which is in there)(default ip = ssh odroid@10.42.0.10). you also want to use the rviz config file available in the goto_box folder. 

You can then vizualize the position of your reference laser. Make sure that you copy the x and y position to the tf transform of the reference laser. 

==>

Determine the laser reference position:
White laser box:				Own PC:
- Run hokuyo laser node 		- Run amcl, scanmatcher and mapserver with map you want to use to navigate ($roslaunch laser_box_measurement measurement_laser_pos.launch)

Run measurement:
White laser box:																		Own PC:
- Run hokuyo laser with tf (roslaunch laser_box_measurement measurement.launch) 		- Run pose measurement and measurement transform (roslaunch laser_box_measurement measurement.launch)
(tf x and y position you just measured with amcl)
