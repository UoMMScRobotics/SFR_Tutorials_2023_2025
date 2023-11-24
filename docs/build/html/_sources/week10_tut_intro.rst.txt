Autonomous Navigation Introduction
================================================

Driving a robot around is fun to begin with, but if you had to drive your robot vacuum cleaner around then the novelty would quickly would wear off.  Autonomous navigation is the answer.  An agent (a human or an algorithm) supplies a goal position, and we expect the robot to do the rest.  When a human supplies goal positions (and makes the decision where these are) this is normally called semi-autonomous navigation.

In previous weeks we have gradually built up a simulation of a robot which can generate a map (or use an existing map) and localise itself in an environment.  These were key stepping stones to autonomous navigation.

For autonomous navigation we need the following capabilities of a mobile robot:

- Locomotion given an input velocity (Kinematics, Actuation, and Control)
- Sensing to avoid obstacles and produce a map (e.g. lidar)
- Where is the robot (Localisation/SLAM and Transformations)
- Start and Goal positions (in the same global frame)
- Draw a path from the start to the goal (Path Planning)
- Drive the robot whilst attempting to Keep the robot on that path (Control and Navigation)

We can sense obstacles (with a simulated lidar), generate a map of the environment to provide a global frame and a reference of obstacles, move a robot based on a desired command velocity, know where the robot is in the global frame.  All that is left is Path Planning and Navigation.

Everything a mobile robot will need is covered by the existing ROS `Nav2 Navigation Stack <https://navigation.ros.org/>`_.

.. Before we get into path planning and navigation, we need a primer on how to represent obstacles and other threats so a path planner and navigator can avoid them.  This will be our first topic: **Costmaps**.

Behaviour Trees and the Navigation Stack
-----------------------------------------

The Nav2 Navigation stack relies on behaviour trees.  If you are familiar with state machines, behaviour trees are a somewhat similar concept.  These will be covered in more detail in Week 11, so these week you can treat it like a black box.

As a very high-level overview, you can consider the behaviour tree as a solution to handling the transitions between different "modes" in navigation.  For example, these modes may include what happens if the robot gets stuck (recovery behaviours).

Build a Package for This Tutorial
----------------------------------

We will utilise a new ROS2 package called ``navigation_demos`` for these activities, which will contain two additional directories ``launch`` and ``config``.

Make a package the usual way:

.. code-block:: console

    cd ~/MY_ROS_WS/src/
    ros2 pkg create navigation_demos --build-type ament_python
    cd navigation_demos
    mkdir launch
    mkdir config

Your setup.py file should have some extra lines added to include the ``launch`` and ``config`` directories:


.. literalinclude:: ../../ros_ws/src/navigation_demos/setup.py
   :language: python
   :emphasize-lines: 2,3,15,16
   :linenos:


A launch file will be built up gradually, whilst we add more config files to our navigation system.  In the ``launch`` directory of the package, create a launch file called ``nav_demo.launch.py``.  You can copy the code below to get you started.

.. literalinclude:: ../../ros_ws/src/navigation_demos/launch/starter_file.launch.py
   :language: python
   :linenos:

Also add the ``bt_nav.yaml`` to the ``config`` directory.  Download the file |bt_nav.yaml|.

.. |bt_nav.yaml| replace::
   :download:`bt_nav.yaml <../../ros_ws/src/navigation_demos/config/bt_nav.yaml>`

Then check everything builds as per usual.

.. code-block:: console

    cd ~/MY_ROS_WS/
    colcon build
    source install/setup.bash

Now we can start the tutorial.
