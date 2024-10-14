# EEEN62021 Software for Robotics Tutorial Material
This repo contains exercises and materials for EEEN60021 Software For Robotics which follow on the work by Dr Murilo M. Marinho (https://ros2-tutorial.readthedocs.io/en/latest/index.html).

The exercises and tutorials can be found here: https://uommscrobotics.github.io/SFR_Tutorials/

## Prerequisites 

* ROS Messages
* ROS Packages
* ROS Nodes, Services & Actions
* ROS Introspection (e.g. RQT)

## Topics Covered in The Material

* Universal Robot Description Files (URDF and RVIZ)
* Robots in Simulation (Gazebo and ROS2 Bridges)
* Perception for Navigation (Maps, Localisation, SLAM, Lifecycle Nodes)
* Autonomous Navigation for UGVs (Nav2 Planners, Controllers, and Costmaps)
* Behaviour Trees (for autonomy)

## How to Contribute to This Tutorial
### Making Edits
1. Clone the repo to a local directory
2. For small changes (e.g. typos) stay on the "main" branch, otherwise make a new branch
3. Make necessary edits (every page is found in /docs/source
4. Use the command ``make html`` in the /docs directory to generate .html files* - these can be viewed on your local machine using a normal browser
5. Check you are happy with your edits
6. When ready push the files back to GitHub

Images are stored in a seperate directory (/figures).  All ROS materials (packages, nodes etc) are in the /ros_ws directory.  This is helpful when you wish to add example files.

*any html files (or anything in the /build directory) are ignored by Git

### Updating Online Material
An automatic GitHub "action" is triggered when a new push to the "main" branch happens.
This builds the html files on the "gh-pages" branch, and these pages then go live.  The gh-pages branch should not be changed manually, as any changes will be wiped during the next push to "main".

Therefore, if you are making lots of big changes, it is sensible to make a temporary branch to make all the edits before merging back to "main".  This avoids having to rebuild the entire website each time.

## FAQs

<details><summary>What is a .rst file?</summary>
  It is a "reStructuredText" file, specifically designed for technical documentation.  It uses primarily Markdown syntax, but html and other commands are available.  Here is a link to some [documentation](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html), but there are plenty of cheatsheets available too.
</details>

<details><summary>How do I add a new page?</summary>
  Create a .rst file (e.g. my_awesome_tutorial.rst), then ensure it has been added to the contents.rst file.  [This page](https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html) has some information on the TOCTree (Table of Contents Tree) command and other things related to including new pages.
</details>

<details><summary>Questions to be addressed</summary>
* Add an image
* Add a hyperlink
* Add a link to another page
* Add a code snippet from a file
</details>
