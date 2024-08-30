Change log
==========

2.2.0 (2024-08-30)
==================

* API changes:
  * Now uses `cmnJointType` instead of `prmJointType`
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * CMake Qt library: added missing library to link on Windows

2.1.0 (2023-11-21)
==================

* API changes:
  * mtsPID:
    * Assumes control is performed on actuators, not joints
    * Removed all code related to actuator to joint coupling
    * Configuration file is now `.cdg` based and uses JSON (was XML)
    * Effort and position limits come from IO level and are retrieved in `Startup` method
* Deprecated features:
  * mtsPID: Removed non-linear gains
* New features:
  * mtsPID:
    * Now uses the velocities sent through `servo_jp` as reference velocities, much better trajectory following!
    * Added support for disturbance observer
  * mtsPID QtWidget:
    * More signals plotted
    * GUI uses CRTK naming convention
    * Added option to save configuration
  * CMake:
    * Updated install targets and debian packages generation
    * Works with ROS2/colcon
* Bug fixes:
  * mtsPID: fixed deadband transitions

2.0.0 (2021-04-08)
==================

* API changes:
  * mtsPID: use CRTK naming convention
  * mtsPID: IO level needs to provide prmStateJoint instead of separate position/velocity/effort
* Deprecated features:
  * Removed mtsTeleoperation, use sawIntuitiveResearchKit mtsTeleopPSM instead
* New features:
  * mtsPID: added feed forward for arms with gravity compensation
  * mtsPID: support prmConfigurationJoint
* Bug fixes:
  * mtsPID: better velocity estimation when in simulation mode

1.7.0 (2019-04-09)
==================

* API changes:
  * Using c++ 14 features **Ubuntu 16.04 or higher** required
  * osaCartesianImpedanceController: added support for deadband
  * mtsPIDQtWidget: removed set all goals to 0 (dangerous jumps)
* Deprecated features:
  * None
* New features:
  * None
* Bug fixes:
  * mtsPID: use number of active joints for mEffortUserCommand
  * mtsPIDQtWidget: fixed unit conversion for joint values, fixed max joint number in plot area

1.6.0 (2018-05-16)
==================

* API changes:
  * JointState objects now maintain the joint type (revolute, prismatic...)
* Deprecated features:
  * Removed `GetJointType`
* New features:
  * Use `CMN_ASSERT` to check vector sizes
* Bug fixes:
  * None

1.5.0 (2017-11-07)
==================

* API changes:
  * mtsPID: full rewrite of main loop using iterators
  * mtsPID: most internal variables renamed, uses prmStateJoint as much as possible
  * mtsPID: throttle tracking error messages
  * mtsPID: now uses mtsMessage for status/warning/error from provided interface
* Deprecated features:
  * None
* New features:
  * Added osaCartesianImpedanceController!
* Bug fixes:
  * mtsPID: desired position properly set when PID is not active on any joint (PID off or in torque mode)
  * mtsPID Widget:https://github.com/jhu-saw/sawRobotIO1394/blob/devel/CHANGELOG.md
    * only sends desired position if in direct mode!
    * added confirmation message when switching to direct mode

1.4.0 (2016-08-31)
==================

* API changes:
  * mtsPID: when in torque mode, update last desired position based on current position
* Deprecated features:
  * None
* New features:
  * CMake: separated components from applications/examples (catkin build 0.4 compatible)
* Bug fixes:
  * None

1.3.0 (2016-01-08)
==================
https://github.com/jhu-saw/sawRobotIO1394/blob/devel/CHANGELOG.md
* API changes:
  * None
* Deprecated features:
  * None
* New features:
  * PID: added SetSimulated mode
  * PID: added methods to enable/disable some joints (used to change actuator coupling on the fly)
  * PID: added joint type in XML configuration, used to be pulled from IO, now needed for simulation
  * PID Qt widget: plot/display desired effort
* Bug fixes:
  * PID: forward error as status when PID is disabled

1.2.0 (2015-10-18)
==================

* API changes:
  * Teleop: added commands to lock position/orientation
  * PID: use prmStateJoint (mapped to ROS joint state message) for communication with Qt and ROS
* Deprecated features:
  * None
* New features:
  * PID: use velocities provided by IO level for better damping
  * PID: provides state joint desired (name/position/effort) and measured (name/position/velocity/effort)
  * PID Qt: plot more information (requested/measured torques), better labels for plot
  * Teleop: force master re-align when camera pedal is release (for SUJ support)
  * Teleop: added options to lock orientation/position
  * Teleop Qt widget: buttons to lock orientation/position
* Bug fixes:
  * Teleop: when aligning master to slave, use trajectory mode
  * PID Qt: fixed conversion to mm

1.1.0 (2015-04-28)
==================

* API changes:
  * Teleop: updated command and event names to match sawIntuitiveResearchKit 1.1.0
  * PID: updated command and event names to match sawIntuitiveResearchKit 1.1.0
* Deprecated features:
  * None
* New features:
  * PID: throw event (Warning) when a requested position is outside joint limits
  * PID: throw event (Error) when tracking error is too high
  * Teleop Qt widget: enable check now reflects component status, added text window for messages
* Bug fixes:
  * Catkin/ROS CMake fixes


1.0.1 (2014-12-22)
==================

* No change log file, initial release.
