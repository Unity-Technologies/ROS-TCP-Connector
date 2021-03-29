# ROS-TCP-Connector

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Installation
1. Inspect the "Tags" tab of the branches drop-down to find your desired version (unless you have a compelling reason to use a different version, we strongly encourage using the most recent).  
<img src="https://user-images.githubusercontent.com/29758400/110989845-57818000-8327-11eb-9e57-e19decc13b4b.png" width=275> </img>  
2. Using Unity 2020.2 or later, open the package manager from `Window` -> `Package Manager` and select "Add package from git URL..."  
![image](https://user-images.githubusercontent.com/29758400/110989310-8ea36180-8326-11eb-8318-f67ee200a23d.png)
3. Enter the following URL, with your desired package version substituted where we've put `v0.2.0` in this example:
`https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector#v0.2.0`
4. Click `Add`


## Tutorials 
Scripts used to send [ROS](https://www.ros.org/) messages to an [TCP endpoint](https://github.com/Unity-Technologies/ROS_TCP_Endpoint) running as a ROS node.

This Unity package provides three main features:

- ROSConnection: See the [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/master/tutorials/ros_unity_integration/README.md) repository for information and tutorials on how to use this component.

- [Message Generation](MessageGeneration.md)

- [ROSGeometry](ROSGeometry.md)

## ROS#

Special thanks to the Siemens [ROS# Project Team]( https://github.com/siemens/ros-sharp/wiki/Info_Acknowledgements) for driving the ROS-Unity Integration Open Source since 2017.

## Community and Feedback

The Unity Robotics projects are open-source and we encourage and welcome contributions. 
If you wish to contribute, be sure to review our [contribution guidelines](CONTRIBUTING.md) 
and [code of conduct](CODE_OF_CONDUCT.md).

## Support
For general questions, feedback, or feature requests, connect directly with the 
Robotics team at [unity-robotics@unity3d.com](mailto:unity-robotics@unity3d.com).

For bugs or other issues, please file a GitHub issue and the Robotics team will 
investigate the issue as soon as possible.

## License
[Apache License 2.0](LICENSE)