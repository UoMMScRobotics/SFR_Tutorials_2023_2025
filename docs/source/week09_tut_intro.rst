Sensing and Perception for Navigation
================================================

Robots are nothing without autonomy.  For mobile robotics a core autonomous capability is navigation, being provided a goal pose and being able to maneuver to that desired location.

This can be performed so-called semi-autonomously, where a human provides the goal, or fully-autonomously where another non-human agent makes the decision on the goal pose (for example, a robot returning to a charging dock when low on battery).

To successfully navigate, mobile robots rely on extroceptive sensing to perceive the environment around them.  This information then needs to be converted into a form that the robot can make use of.

Sensor Messages
---------------------

ROS provides established sensors messages to link common sensors into our robots.  The documentation can be found `here <https://docs.ros.org/en/humble/p/sensor_msgs/>`_.

For navigation, primarily `LaserScan <https://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/LaserScan.html>`_ (2D lidar) and `PointCloud2 <https://docs.ros.org/en/humble/p/sensor_msgs/interfaces/msg/PointCloud2.html>`_ (depth cameras and 3D lidar) are used for situational awareness during navigation.
These sensors have become incredibly affordable.  If you canvas common robotic platforms you will observe a very common setup of a 2D or 3D lidar for long range obstacle detection and depth cameras for nearby the robot (e.g. looking for clutter on the floor).

.. NOTE::

    You can of course define your own custom messages!  This has been the case for radiation sensors, which have their own message format.
    See the `Paper <https://research.manchester.ac.uk/files/223331152/ROS_Messages_for_Nuclear_Sensing.pdf>`_ and `GitHub <https://github.com/EEEManchester/radiation_msgs>`_

Interpretation of Sensor data
------------------------------

For navigation (in 2D) obstacles are represented in a map.  In it's simplest form, a map is essentially a picture where the colour of the pixels indicates if an area has an obstacles (can't travel through), is free space (can travel through) or unknown (yet to be observed, may or may not be allowed to travel through).  It is these maps we will concentrate on for the tasks ahead.

There are other forms of representation, such as the voxel grid, which extends the grid based map idea to voxels (cube shaped cells) in three dimensions.  These voxels also have the same three values of occupied, free, or unknown.