Conclusion
===========

**Topics in this Tutorial**
- Running a robot (simulation) with SLAM
- Adding sensor data and an occupancy grid map to make costmaps
- Autonomous navigation using the ROS 2 Navigation Stack
- How to find a planner and controller suitable for a physical robot

We have seen how the ROS2 Navigation Stack provides the functionality to have a robot semi-autonomously navigate to a goal pose.

The robot needs to be able to localise itself in an environment, sense threats, plan a path and control its state to follow that path.

Our choice of path planner and controller is based on functionality, the type of robot actuation, and any other requirements.

Note there are additional plugins such as the `Rotation Shim Controller <https://navigation.ros.org/configuration/packages/configuring-rotation-shim-controller.html>`_ and `Smoothers <https://navigation.ros.org/configuration/packages/configuring-smoother-server.html>`_ (both paths and velocities) along side our chosen planner and controller plugins.  This can further augment how our robot behaves in certain situations for better or worse.

With real robots and sensing, tuning of controllers and other plugins becomes vital.  Don't be scared to read the documentation or academic papers, adjust as necessary.  You can always change the values back!