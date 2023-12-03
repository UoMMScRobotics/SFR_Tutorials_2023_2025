Conclusion
===========

**Topics in this Tutorial**

- Using Groot to build and modify behaviour trees visually and the underlying .xml description file
- Generating a simple behaviour tree using existing Nav2 nodes
- Modifying behaviour trees to include additional functionality (in our case replanning)

We have seen how the ROS2 Navigation Stack provides the functionality to have a robot semi-autonomously navigate to a goal pose, but the behaviour tree portion was glossed over.  The behaviour tree is in fact what dictates the order and the capabilities of a robot during navigation (for the ROS navigation stack).

There are existing behaviour trees available for navigation which should serve most purposes, however, behaviour trees are not only useful for navigation.  They can be used to design entire autonomous missions, for example in the book "A Concise Introduction to Robot Programming with ROS" by F. M. Rico they create a patrolling example, a video of which can be seen below.

.. raw:: html

   <iframe width="560" height="315" src="https://www.youtube-nocookie.com/embed/lStwIxNIPBc?si=tHyP375JU6oJ8l2D" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

The BehaviourTree.CPP library allows us to write our own tree nodes in C++, and add any behaviour trees to our robots.


If you wish to check your code, or are having issues, you can download an example version of the package generated in this tutorial. |bt_demos.zip|

.. |bt_demos.zip| replace::
   :download:`bt_demos.zip <../../ros_ws/src/bt_demos.zip>`