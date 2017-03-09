weiss_kms40
=========================================

ROS Driver for the [force-torque-sensor KMS40](https://www.weiss-robotics.com/en/produkte/force-torque-sensing/kms-40-en/) of Weiss robotics

Publishes a [Wrenched Stamped topic](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html) **/wrench**

Emulator of geometry_msgs/WrenchStamped with rqt-plugin

## Usage

```roslaunch weiss_kms40 kms40.launch```

Use ```rqt --force-discover``` after catkin_make to make plugin visible


## Acknowledgements
This project is a result of the LIAA project.
http://www.project-leanautomation.eu/

![LIAA](http://www.project-leanautomation.eu/fileadmin/img/LIAALogo/Logo_LIAA.png "LIAA")

![EC](http://www.project-leanautomation.eu/typo3temp/pics/b3ba71db31.jpg "EC")

LIAA received funding from the European Union’s Seventh Framework Programme for research, technological development and demonstration under grant agreement no. 608604.

Project runtime: 02.09.2013 – 31.08.2017.