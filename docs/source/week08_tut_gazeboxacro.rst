Adding Simulation Properties
=============================

Open the file ``/urdf/diff_drive.urdf.xacro`` and take a brief look at the contents. Notice that links have ``<interial>`` and ``<collision>`` tags - this makes for quite a lengthy looking file.  The interia matrix can be found online for simple geometries (sphere, box, cylinder), or if you have a mesh file you can find the inertia matrix from CAD software or software like `Meshlab <https://www.meshlab.net/>`_ or `Cloudcompare <https://www.danielgm.net/cc/>`_.  These are critical for the Gazebo physics engine to handle things properly.

This is also the case for real robots!  Manipulators for example often need an accurate estimate of the centre of mass and sometimes interia matrix to handle an end-effector or payload correctly.  These were touched upon when looking at :doc:`URDF files in general <../week07_tut_collisions>`, this tutorial will focus on additions needed for simulation.

.. Note::
    Using xacro allows for variables to be used (and reused) in urdf files, however, the ``check_urdf`` helper does not support xacro macros.  To get around this, use the xacro package to convert to a pure .urdf format:

    ``source <WORKSPACE>/install/setup.bash
    cd <WORKSPACE>/src/<PACKAGE>/<URDF_DIR>/
    check_urdf <(ros2 run xacro xacro <FILENAME>.urdf.xacro)``

    The ``xacro xacro`` command performs all the substitutions, the output is piped into ``check_urdf`` all in one go.  As xacro needs some ROS functionality it is necessary to source the workspace prior to running ``xacro xacro``.

Adding Gazebo Information
----------------------------

At the bottom of ``/urdf/diff_drive.urdf.xacro``, on line 213 we include another file called ``/urdf/example_gazebo.xacro``.  To make things easier to read, this will contain all the necessary components for simulation.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/diff_drive.urdf.xacro
    :language: xml
    :lines: 213
    :lineno-start: 213
    :emphasize-lines: 1
    :linenos:

Open the ``example_urdf_robot/urdf/example_gazebo.xacro`` file in your text editor of choice.  We will walk through each portion individually.


Joint State Publisher
-------------------------------

The joint state publisher provides regular publishing of the joint information from the simulation.  In this case, it would be the angular velocity of the left and right wheels.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 1-18
    :linenos:

By default, all (moveable) joint states are published by Gazebo when we include the `JointStatePublisher <https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1JointStatePublisher.html>`_ plugin.  If we remove these two lines defining the joints we are interested in, it will still work.  However, for completeness specifying the joints has been included.  Defining joints manually is good practice to avoid unnecessary use of bandwidth.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 12-13
    :lineno-start: 12
    :emphasize-lines: 1-2
    :linenos:


Differential Drive Controller
-------------------------------

Gazebo offers a plugin for a differential drive controller.  It takes in arguments regarding wheel radius and wheel separation, and provides the necessary wheel angular velocities - how helpful!


.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 19-45
    :lineno-start: 19
    :linenos:

Firstly, we tell Gazebo which moveable joints are part of the differential drive system - in this case the left and right wheels.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 24-26
    :lineno-start: 24
    :emphasize-lines: 2-3
    :linenos:

Secondly, we give the kinematics to calculate motor velocities correctly.  Notice that as a xacro, it is possible to use variables from the main .urdf.xacro file  (e.g. ``${wheel_radius}``) - super helpful!.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 28-30
    :lineno-start: 28
    :emphasize-lines: 2-3
    :linenos:

Thirdly, we setup information regarding transformations and estimates of robot position.  The `odom` information is the estimate the robot has of its pose using dead reckoning.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 32-36
    :lineno-start: 32
    :emphasize-lines: 1,4-5
    :linenos:

The ``odom_publish_frequency`` is simply how often we want the differential drive controller to provide the dead reckoning odometry estimate.  The ``<frame_id>`` and ``<child_frame_id>`` are simply the frame names for where the robot starts and where to connect the transform to.


.. WARNING::
    Remember, despite being a simulation, issues such as wheel slip will mean that the odometry estimate *will* accumulate errors.  There are methods to extract the exact pose by different means in Gazebo if you require a ground truth pose estimate.

Finally, we can specify topic names explictly as listed in `DiffDrive <https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html>`_ documentation.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 38-42
    :lineno-start: 38
    :emphasize-lines: 2-5
    :linenos:


.. Note::
    In this example, we have omitted parameters such as:
    * min/max jerk (linear and angular)
    * min/max acceleration (linear and angular)
    * min/max velocity (linear and angular)

    All these can be found in the `DiffDrive <https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html>`_ documentation.

    As a "best practice" approach, it is usually sensible to add acceleration and velocity limits.  This applies to real robots too, to avoid "jerky" motions.


Sensors
-------------
Gazebo offers a range of sensors which can be included in simulation.  Note that not all sensors have been implemented yet (see `comparison chart <https://gazebosim.org/docs/fortress/comparison/>`_), but most are available.  Furthermore, it is possible to write your own plugins for sensors or noise models, but that is beyond the scope of this tutorial.

Sensors must be attached to a coordinate frame using the ``<frame_id>``.  For sensors such as cameras or lidars, usually there exists an link defined in the URDF.  However, for sensors such as an IMU these are arbitrarily attached to the robot chassis without the need for a specific link.  Sensors also need an ``<update_rate>`` tag to state how often they will publish data.

Lidar
^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 48-84
    :lineno-start: 48
    :linenos:

The individual lines will not be discussed in detail, but this offers you an example of how a Lidar sensor can be implemented.  It also demonstrates how you can define properties such as the min/max range of the sensor or how many beams it has.

.. Note::
    This sensor has been based on the `SICK TiM550 <https://www.sick.com/tr/en/catalog/products/lidar-and-radar-sensors/lidar-sensors/tim/tim551-2050001/p/p343045?tab=detail>`_ series of 2D lidar.  The link shows the technical information of the sensor.


IMU
^^^^^^^^^^^^^^^^^^^^^^

Inertial measurement units are ubiquitous, therefore, it is expected that nearly all mobile robots will have one (less so for manipulators).

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 87-153
    :lineno-start: 87
    :linenos:

Notice that for each axis of the gyro and accelometer (six in total), there is a ``<noise>`` tag.  Once again, characterisation of real sensor hardware can tell us what these values *should* be in our simulation.

General physics
-------------------

For our robot to have the wheels stick to the floor, but the castors slide, the coefficients of friction need to modified.

.. literalinclude:: ../../ros_ws/src/example_urdf_robot/urdf/example_gazebo.xacro
    :language: xml
    :lines: 155-177
    :lineno-start: 155
    :linenos:

The front and rear castors have their friction values set to a very small value (1e-5), whereas the wheels have a large coefficient value.  The actual definitions of ``<mu>`` and ``<mu2>`` are not important for this example, but more details can be found `here on GitHub <https://github.com/osrf/gazebo_tutorials/blob/master/friction/tutorial.md>`_, `here on Gazebo Classic docs <https://classic.gazebosim.org/tutorials?tut=friction>`_ and `here on a third party forum <https://www.althack.dev/ignition_vs_gazebo/friction/#>`_.  As a general rule, the two values should be identical.


Now we have examined the gazebo plugins.  The next section will run the simulation and explore how to link it into ROS more closely.


.. ign topic --pub "linear {x:0.5}" --topic /model/diff_drive_example/cmd_vel --msgtype ignition.msgs.Twist
.. ros2 run teleop_twist_keyboard teleop_twist_keyboard
