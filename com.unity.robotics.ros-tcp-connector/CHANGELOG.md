# Changelog

All notable changes to this repository will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](http://semver.org/spec/v2.0.0.html).

## Unreleased

Add the [Close Stale Issues](https://github.com/marketplace/actions/close-stale-issues) action

### Upgrade Notes

Upgrade the TestRosTcpConnector project to use Unity LTS version 2020.3.11f1

### Known Issues

### Added

Add the Ros Tcp Connector assembly to support Universal Windows Platform

Added the CameraInfoGenerator that takes a Unity Camera and a provided HeaderMsg, generate a corresponding CameraInfoMsg, see:
[CameraInfo Generator](https://github.com/Unity-Technologies/ROS-TCP-Connector/issues/133)

### Changed
- Publishing a message to an unregistered topic will show an error.

### Deprecated

### Removed

### Fixed
  - Fixed the issue when queuing a message fails if the type is unspecified in compile type.


## [0.5.0-preview] - 2021-07-15

### Upgrade Notes

Update third party notices also

### Known Issues

### Added

Add the option to connect to ROS2. User can use a dropdown menu and choose ROS1 or ROS2 protocols to connect to the ROS side

Add badges to main README

### Changed

### Deprecated

### Removed

### Fixed

Fixed byte conversion method in the serialization process

## [0.4.0-preview] - 2021-05-27

Note: the logs only reflect the changes from 0.3.0-preview

### Upgrade Notes

RosConnection 2.0: maintain a single constant connection from Unity to the Endpoint. This is more efficient than opening one connection per message, and it eliminates a whole bunch of user issues caused by ROS being unable to connect to Unity due to firewalls, proxies, etc.

### Known Issues

### Added

Add a link to the Robotics forum, and add a config.yml to add a link in the Github Issues page

Add connection status lights to the HUD - blue if ok, bright blue if actively sending, red if there's a problem. Turning off "Connect on Startup" will allow to set the IP to connect

Add lint and test coverage reporting

### Changed

Reduce character count for path to generated messages. The folder `Runtime/MessageGeneration/PregeneratedMessages` is moved to the parent directory and renamed `Runtime/Messages`

### Deprecated

### Removed

### Fixed

Correct the namespace for the MDuration class
