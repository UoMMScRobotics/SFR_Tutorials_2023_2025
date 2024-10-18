Localisation in a Map
============================

Picture this, you are walking/driving around and trying to figure out where you are.  You might look at road names, landmarks, signs to other points of interest.  This process of figuring out where you are is called localisation.

Imagine our robot is in a known environment, with an existing map, it must localise itself in the map to be able to navigate.  For this exercise, a robot will be spawned into the Turtlebot3 simulation environment, and localised against the existing map provided by the map server.

Inspecting the Launch File
-----------------------------

Once again, before launching any nodes, the launch file will be inspected to understand how it works.

.. literalinclude:: ../../ros_ws/src/example_gz_robot/launch/localisation.launch.py
    :language: xml
    :lines: 9-21
    :lineno-start: 9
    :linenos:
    :emphasize-lines: 8-12

A launch file called ``simulation_bringup.launch.py`` has been provided for this exercise.  This launch file is included in this portion.

.. literalinclude:: ../../ros_ws/src/example_gz_robot/launch/localisation.launch.py
    :language: xml
    :lines: 22-34
    :lineno-start: 22
    :linenos:
    :emphasize-lines: 2,3,11

This should look familiar from the last exercise.  It simply publishes the map as a ros topic based on the .pgm and .yaml file.

.. literalinclude:: ../../ros_ws/src/example_gz_robot/launch/localisation.launch.py
    :language: xml
    :lines: 35-43
    :lineno-start: 35
    :linenos:

To actually provide the localisation, this exercise uses AMCL (`Adaptive Monte Carlo Localisation <http://wiki.ros.org/amcl>`_).  It uses scan matching to estimate the robot position based on the map and lidar scan data, however, the actual algorithm is an extension of the particle filter.

The lifecycle manager once again makes life easier for us, ensuring that AMCL will not start without the map being published.

.. literalinclude:: ../../ros_ws/src/example_gz_robot/launch/localisation.launch.py
    :language: xml
    :lines: 54-75
    :lineno-start: 54
    :linenos:

Finally, we use RViz to see the map, the robot, and sensor data.

Using Localisation
-----------------------

Run the launch file with,

.. code-block:: console

    source ~/<YOUR_ROS_WS>/install/setup.bash
    ros2 launch example_gz_robot localisation.launch.py

Once everything has loaded, you will immediately notice in RViz (and the terminal) that the robot has not been localised in the map at all!  This is because AMCL needs an initial guess, published on the ``/initialpose`` topic, consisting of a geometry_msgs/msg/PoseWithCovarianceStamped.

It is diffcult to figure out the coordinates and orientation of the robot in the terminal.  RViz offers a helpful ``2D Pose Estimate`` button, this allows us to put a large green arrow down approximately where the robot is, and set the orientation.  This can be performed multiple times and AMCL will take a new guess each time, just in case the estimates are poor.

.. list-table::
   :width: 100%
   :class: borderless

   * - .. image:: ../../figures/week09/localisation_pose_estimate.png
          :width: 100%
          :alt: Using the 2D Pose Estimate button in RViz.
          :align: center
         
     - .. image:: ../../figures/week09/initial_amcl.png
          :width: 100%
          :alt: The uncertain initial pose estimate by AMCL
          :align: center

The image to the right shows the estimate of the robot's pose given by AMCL, along with the lidar scan in red.  The cluster of green arrows are a visualisation of the particle filter at work.  For the initial guess, the spread of candidate points is large, but as the robot moves around, the localisation estimate will generally improve.


Improve the Localisation Estimate
----------------------------------

This portion will have you drive the robot around and observe the improvement (or possible degradation) of the estimated robot pose.  In a terminal window, use the command

.. code-block:: console
    
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

.. DANGER::
    
    Pay attention to the lidar scan, and use that to avoid any obstacles.  The estimate of the robot pose in the map may be poor, so you can not rely on it.  The lidar scan will be relative to the robot, so should always be a good indicator of an imminent collision.



Drive the robot around, and the cluster of green arrows should become more tightly packed together.  This is a visual representation of the confidence in the estimate increasing.  However, you may still notice the odd arrow in certain places, particularly around the central pillars where it might be difficult to distinguish between different pillars.  A clear indication of better localisation comes from the lidar scan, which should overlay very well with the map.

Once you are happy you have witnessed the power of particle filters for localisation, stop all processes with ``Ctrl+C``, close all terminals, and move on to the next section.
