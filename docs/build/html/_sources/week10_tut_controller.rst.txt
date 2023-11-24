Adding a Controller from Nav2
================================================

As stated previously, the ``Controller`` is the magic that gets a robot to move, and governs a lot of the behaviour of how it navigates.  It is also the most fiddly thing that will require tuning on a real robot to work reliably.

For a full list of ``Controllers`` available, visit the Nav 2 Controller `Documentation <https://navigation.ros.org/plugins/index.html#controllers>`_.

The Controller
---------------

The ``Controller`` monitors the robot's state (velocity, pose), the given path, and sensor measurements to compute velocity commands that lead to progress along the path.

Once again we have list of "plugins" to choose from, allowing us flexibility over what controller architecture to use.  In fact, you can write your `own controller <https://navigation.ros.org/plugin_tutorials/index.html>`_ (and planner)!

For this task we will stick with ``DWB`` planner, which is the general default choice.

.. note::
  The name ``DWB`` is a inside joke made by David Lu.  The algorithm used is called "Dynamic Window Approach" (DWA), however, as it was rewritten from ROS 1 to ROS 2, the author thought themselves hilarious to call it ``DWB`` (as in, what comes after "A").

.. note::
    The original DWA approach is based on the paper:
    `Fox, D., Burgard, W. and Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance <https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf>`_.
    
    I told you that all these packages are based on research papers.


As mentioned, these ``Controllers`` can be tuned or altered to provide the behaviour you desire (e.g. how faithfully should it stick to the global path).

Writing the Controller Config File
```````````````````````````````````

The format of the configuration is taken from the `controller documentation <https://navigation.ros.org/configuration/packages/configuring-controller-server.html#configuring-controller-server>`_.  It consists of general controller parameters, but controller plugin specific parameters can also be passed.  For DWB some of these parameters can be `found here <https://navigation.ros.org/configuration/packages/dwb-params/controller.html>`_.  An example config file for the ``DWB`` controller would look like the file below.  Copy this example into a file called ``controller.yaml`` in the ``config`` directory.

.. literalinclude:: ../../ros_ws/src/navigation_demos/config/controller_only.yaml
   :language: xml
   :linenos:

The ``DWB`` planner has many options to tune the system.  It is recommended that for your own robot, you would carefully read about all the different options and see which need to be altered.

Notice as well we needed to include a ``Goal Checker`` and a ``Progress Checker``, these do pretty much what you would expect.  By having these separate (rather than having them included in the velocity controller), it again allows for modularity.  These can be tuned as well, for example, in the goal checker we set the tolerance of how close the robot must be to the goal (as it is nearly impossible for a robot to drive exactly to the point asked of it).


Writing the Local Costmap Config File
```````````````````````````````````````
The Local costmap in comparison to the global costmap, only covers a small portion around the robot (which is quicker to compute), and is updated more regularly.  In this case, we do not necessarily need the static map, just obstacles and other threats we wish to track.  Because we are using *layered* costmaps, it is possible to add obstacle (or other types) of layers representing different sensors.  We'll just add the lidar for obstacles this time.

Again we include the inflation layer to convert obstacles into the configuration space of the robot.  Edit the ``controller.yaml`` file where the ``DWB`` parameters were added earlier to match the example below.

.. literalinclude:: ../../ros_ws/src/navigation_demos/config/controller.yaml
   :language: xml
   :emphasize-lines: 63-95
   :linenos:


Adding a Controller to a Launch File
--------------------------------------

Finally, we need to add the node to the launch file, along with including the config file and ensuring the lifecycle manager knows to handle it.


.. literalinclude:: ../../ros_ws/src/navigation_demos/launch/nav_demo.launch.py
   :language: python
   :emphasize-lines: 20, 29, 100
   :linenos:


Perform the usual ``colcon build``, ``source install/setup.bash`` and check the launch file runs.


