Describing Robots using URDF
=================================

Given a set of joint rotations or translations it is possible to know the configuration of system.  Furthermore, knowing all the link lengths etc means this information can be fed into an algorithm to know the forward kinematics or compute *inverse* kinematics.  An even more common use case is knowing how to transform a reading from a sensor (e.g. distance from a lidar) into a different coordinate frame.

The `TF2 package <https://wiki.ros.org/tf2>`_ supports computation of rigid body transformations (very helpful!), but it needs to know about all the links and joints of a robot.  A "robot description" is a method to achieve this.

The **Universal Robot Description File (URDF)** is a solution to this problem.  It is a .xml file which describes joints and links for a robot.

Viewing a Robot Description
----------------------------

There are specific extensions for IDEs or websites to view URDF files, however, the most convenient is the built in RVIZ visualisation tools that ROS ships with.

Install the `urdf_launch <https://index.ros.org/p/urdf_launch/github-ros-urdf_launch/#humble>`_ package to easily enable this.

.. code-block:: console

    sudo apt install ros-humble-urdf-launch

It can be invoked using ``ros2 launch urdf_launch display.launch.py`` with some extra parameters, we will use this later.

URDF format and XML
--------------------

Before jumping into writing a URDF file, let's take a quick look at the format.

XML uses tags to "open" and "close" certain blocks of information, for example:

.. code-block:: XML

    <tag>"information"</tag>

The tag text needs to match, and the use of the leading forward slash ``/`` denotes it is a closing tag.  Tags can have child elements, therefore, you often see nested tags, with indentation.

.. code-block:: XML

    <tag>
      <element>"information"</element>
    </tag>

Finally, you will most likely see an element being *specifically* referenced inside a tag, this is instead called an "attribute".

.. code-block:: XML

    <tag attribute="information"/>

The use of a forward slash at the end saves explicitly writing an additional tag - coders are lazy after all.


URDF Element - Joint
--------------------

The ``joint`` tag describes a fixed (doesn't move), revolute (rotates between limits), continuous (can rotate without limits, e.g. wheel), prismatic (linear movement), and a couple other options.


.. image:: ../../figures/week07/joint.png
  :width: 400
  :alt: Example joint from `urdf docs <http://wiki.ros.org/urdf/XML/joint>`_.
  :align: center 

You can imagine that joints provide all the rigid transformations between each frame in the robot's default state.

::

    joint
    ├── name
    ├── type
    ├── parent
    ├── child
    ├── origin (optional)
    |   ├── xyz
    |   └── rpy
    ├── ... (more optionals)

All the optional elements such as joint limits (for revolute or prismatic) can be found in the `documentation <http://wiki.ros.org/urdf/XML/joint>`_.  How that actually looks for a fixed joint is:

.. code-block:: XML

    <joint name="example_fixed_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.5 0 0" rpy="0 0 0"/>
    </joint>

Distances are in metres, angles are in radians.  This represents a camera which is placed 0.5 m above a "base_link" frame.  The base_link frame is the normal name to represent the centre of the body of a mobile robot or fixed point for a static manipulator.

Let's build our first robot!