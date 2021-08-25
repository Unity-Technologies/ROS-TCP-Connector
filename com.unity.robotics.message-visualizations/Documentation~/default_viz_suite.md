# Message Visualizations

The message visualizations package enables Unity projects to visualize incoming and outgoing information from ROS, such as sensor data, navigation messages, markers, and more. This package provides default configurations for common message types, as well as APIs to create custom visualizations.

- [Message Visualizations](#message-visualizations)
  - [Getting Started](#getting-started)
  - [Configuring a Visualization Suite](#configuring-a-visualization-suite)
  - [The HUD](#the-hud)
    - [](#)
  - [Using the Inspector](#using-the-inspector)
    - [Message Topics](#message-topics)
    - [TF Topics and Tracking](#tf-topics-and-tracking)
    - [Visualization Settings](#visualization-settings)
    - [Pointclouds](#pointclouds)

## Getting Started

This package contains a `DefaultVisualizationSuite` prefab that provides visualizer components for many common ROS message types. These components control how messages are displayed in the Unity scene.

1. To begin, you will need to have ROS—Unity Integration set up in your project in order to send and receive ROS messages. If this is not yet set up, follow the steps [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/README.md).
2. If you have not already added the Message Visualizations package in your Unity project, follow the [Installation steps](../../README.md#Installation) for the package.
3. Open the Unity scene that you would like add message visualizations to. In the Project window, expand `Packages/Message Visualizations`. Select the `DefaultVisualizationSuite` (indicated by the blue cube Prefab icon) and drag it into your scene Hierarchy.

    ![](../images~/VizPrefab.png)

4. Enter Play mode. The heads-up display (HUD) panel in the top left indicates a successful connection via the colored arrows in the top-left corner. If the HUD is not visible, ensure your connection throws no errors, and that `Show HUD` in the ROS Settings is on. More information about the HUD can be found [below](#the-hud).

    > If you encounter networking issues, please refer to the [Networking Guide](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/network.md).

5. Once connected, you may begin sending and receiving ROS messages as usual. Topics will by default populate in the HUD panel's `Topics` list. For this example, run a simple publisher from the commandline:
    ```
    rostopic pub /point geometry_msgs/Point 1 2 3
    ```

    Click the `Topics` tab in the HUD. You will see this sample topic, `/point` in the list. Click the topic name to toggle both the `UI` and `Viz` (alternatively, you can click each individual toggle).

    > The visualization is created based on the message type. You can also explicitly set a visualizer's topic in its [Inspector](#using-the-inspector).

6. You should now see a new window labeled with the topic in your Game view, populated with the `geometry_msgs/Point` data published. Additionally, the point is being drawn in the scene. You've now successfully set up visualizations!

    ![](../images~/VizPoint.png)


## Configuring a Visualization Suite

The [Getting Started](#getting-started) steps describes the `DefaultVisualizationSuite` prefab that provides visualizer components for many common ROS message types. You may also create your own visualization suite by creating a GameObject with only the desired default or custom visualizer components for your project.

The UI windows for visualizations can be dragged and resized. The visualizations can be customized as described in the [Inspector](#using-the-inspector) section. The topics being visualized and the window configurations are saved between sessions and can be exported and shared via the HUD's `Layout > Export/Import layout` buttons.

## The HUD

![](../images~/VizHud.png)

The interactive buttons on the HUD panel includes:
- Topics: Contains a list of all subscribed ROS topics in this current session. The `UI` toggle enables a window that shows the last message sent or received on that topic. The `Viz` toggle enables an in-scene drawing that represents the last message sent or received on that topic. If no `Viz` toggle is enabled, that topic does not have a default visualizer enabled in the Unity scene.
- Transforms: Contains [`tf`](http://wiki.ros.org/tf) visualization options, including displaying the axes, links, and labels for each frame.
- Layout: Contains options to save and load this visualization configuration. While the visualization components are by default saved via the scene or the prefab, the window layout and visualized message list is saved as a JSON file. By default, this file is saved to a `RosHudLayout.json` file on your machine's [`Application.persistentDataPath`](https://docs.unity3d.com/ScriptReference/Application-persistentDataPath.html). In this Layout tab, you can choose to `Export` this JSON file with a custom name to a chosen location on your device, as well as `Import` a layout JSON file to begin using that saved visualization configuration.
- Markers: TODO

## Using the Inspector

### Message Topics

TODO: how to subscribe to a topic properly

### TF Topics and Tracking

TODO: how to subscribe to a TF topic properly, how to set tracking type, what do these types mean

**Exact**

**Track Latest**

**None**

### Visualization Settings

TODO: color, size

> Note: The "radius" field is in Unity coordinates, where 1 unit = 1 meter.

### Pointclouds

Similar to the Visualization Settings, pointcloud visualizations are highly customizable. Settings for these message visualizers (PointCloud, LaserScan, etc.) will be saved during runtime. For more information on this, you can check out the [base SettingsBasedVisualizer]() class, as well as read more about Unity's [ScriptableObjects](). TODO

TODO: pointcloud viz settings