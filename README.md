# ROS TCP Connector

[![Version](https://img.shields.io/github/v/tag/Unity-Technologies/ROS-TCP-Connector)](https://github.com/Unity-Technologies/ROS-TCP-Connector/releases)
[![License](https://img.shields.io/badge/license-Apache--2.0-green.svg)](LICENSE.md)
![ROS](https://img.shields.io/badge/ros-melodic-brightgreen)
![ROS](https://img.shields.io/badge/ros-noetic-brightgreen)
![ROS](https://img.shields.io/badge/ros2-foxy-brightgreen)
![Unity](https://img.shields.io/badge/unity-2020.2+-brightgreen)

## Introduction

This repository contains two Unity packages: the ROS TCP Connector, for sending/receiving messages from ROS, and the Message Visualizations, for adding visualizations of incoming and outgoing messages in the Unity scene.

## Installation
1. Using Unity 2020.2 or later, open the Package Manager from `Window` -> `Package Manager`.
2. In the Package Manager window, find and click the + button in the upper lefthand corner of the window. Select `Add package from git URL....`

    ![image](https://user-images.githubusercontent.com/29758400/110989310-8ea36180-8326-11eb-8318-f67ee200a23d.png)

3. Enter the git URL for the desired package. Note: you can append a version tag to the end of the git url, like `#v0.4.0` or `#v0.5.0`, to declare a specific package version, or exclude the tag to get the latest from the package's `main` branch.
    1. For the ROS-TCP-Connector, enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`.
    2. For Message Visualizations, enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.message-visualizations`.
4. Click `Add`.

To install from a local clone of the repository, see [installing a local package](https://docs.unity3d.com/Manual/upm-ui-local.html) in the Unity manual.

## Tutorials
<!-- Scripts used to send [ROS](https://www.ros.org/) messages to an [TCP endpoint](https://github.com/Unity-Technologies/ROS_TCP_Endpoint) running as a ROS node. -->

This Unity package provides four main features:

- ROSConnection: A component that sets up communication between ROS and Unity. See the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md) for information and tutorials.

- [Message Generation](MessageGeneration.md): A tool to generate C# classes for ROS message types.

- Message Visualization: A suite of default configurations and APIs to visualize incoming and outgoing information from ROS.
    - See the [TEMP link] [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/amanda/viz/default-tutorial/tutorials/message_visualization/default_viz_suite.md) for tutorials on using the Message Visualizations package for visualizing rosbag playback, visualizing custom messages, and more!
    - You can also view the package's [Usage Information](com.unity.robotics.message-visualizations/Documentation~/README.md) for more details on using the package in your own project.

- [ROSGeometry](ROSGeometry.md): A set of extensions that convert geometries between Unity and other coordinate frames.

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