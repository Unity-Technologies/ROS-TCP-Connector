# ROS-TCP-Connector

[![Version](https://img.shields.io/github/v/tag/Unity-Technologies/ROS-TCP-Connector)](https://github.com/Unity-Technologies/ROS-TCP-Connector/releases)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)
![ROS](https://img.shields.io/badge/ros-melodic-brightgreen)
![ROS](https://img.shields.io/badge/ros-noetic-brightgreen)
![ROS](https://img.shields.io/badge/ros2-foxy-brightgreen)
![Unity](https://img.shields.io/badge/unity-2020.2+-brightgreen)

## Installation
1. Using Unity 2020.2 or later, open the package manager from `Window` -> `Package Manager` and select "Add package from git URL..."
![image](https://user-images.githubusercontent.com/29758400/110989310-8ea36180-8326-11eb-8318-f67ee200a23d.png)
2. Enter the following URL. If you don't want to use the latest version, substitute your desired version tag where we've put `v0.5.0` in this example:
`https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.5.0`
3. Click `Add`.


## Tutorials
Scripts used to send [ROS](https://www.ros.org/) messages to an [TCP endpoint](https://github.com/Unity-Technologies/ROS_TCP_Endpoint) running as a ROS node.

This Unity package provides three main features:

- ROSConnection: See the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md) repository for information and tutorials on how to use this component.

- [Message Generation](MessageGeneration.md)

- [ROSGeometry](ROSGeometry.md)

## ROS#

Special thanks to the Siemens [ROS# Project Team]( https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements) for driving the ROS-Unity Integration Open Source since 2017.

## Community and Feedback

The Unity Robotics projects are open-source and we encourage and welcome contributions.
If you wish to contribute, be sure to review our [contribution guidelines](CONTRIBUTING.md)
and [code of conduct](CODE_OF_CONDUCT.md).

## Support
For questions or discussions about Unity Robotics package installations or how to best set up and integrate your robotics projects, please create a new thread on the [Unity Robotics forum](https://forum.unity.com/forums/robotics.623/) and make sure to include as much detail as possible.

For feature requests, bugs, or other issues, please file a [GitHub issue](https://github.com/Unity-Technologies/ROS-TCP-Connector/issues) using the provided templates and the Robotics team will investigate as soon as possible.

For any other questions or feedback, connect directly with the
Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

## License
[Apache License 2.0](LICENSE)