# Message Visualizations

The message visualizations package enables Unity projects to visualize incoming and outgoing information from ROS, such as sensor data, navigation messages, markers, and more. This package provides default configurations for common message types as well as APIs to create custom visualizations.

Visit the [TEMP link] [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/amanda/viz/default-tutorial/tutorials/message_visualization/default_viz_suite.md) to get started with Message Visualizations tutorials.

> This package is compatible with ROS 1 and ROS 2, and Unity versions 2020.2+.

**Table of Contents**
- [Installation](#installation)
- [Configuring a Visualization Suite](#configuring-a-visualization-suite)
- [The HUD](#the-hud)
- [Using the Inspector](#using-the-inspector)
    - [Message Topics](#message-topics)
    - [TF Topics and Tracking](#tf-topics-and-tracking)
    - [Visualization Settings](#visualization-settings)
    - [Joy Messages](#joy-messages)
    - [More on Point Clouds](#more-on-point-clouds)

---

## Installation

1. Using Unity 2020.2 or later, open the Package Manager from `Window` -> `Package Manager`.
2. In the Package Manager window, find and click the + button in the upper lefthand corner of the window. Select `Add package from git URL....`

    ![image](https://user-images.githubusercontent.com/29758400/110989310-8ea36180-8326-11eb-8318-f67ee200a23d.png)

3. Enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.visualizations`.

    > Note: you can append a version tag to the end of the git url, like `#v0.4.0` or `#v0.5.0`, to declare a specific package version, or exclude the tag to get the latest from the package's `main` branch.

4. Click `Add`.

## Configuring a Visualization Suite

This package contains a `DefaultVisualizationSuite` prefab that provides visualizer components for many common ROS message types, organized in the hierarchy by package. These components control how messages are displayed in the Unity scene. You may also create your own visualization suite by creating a GameObject with only the desired default or custom visualizer components for your project.

The UI windows for visualizations will automatically be laid out as they are turned on, but they can also be dragged and resized. The visualizations in the scene can be customized as described in the [Inspector](#using-the-inspector) section. The topics being visualized and the window configurations are saved between sessions and can be exported and loaded via the HUD's `Layout > Export/Import layout` buttons.

## The HUD

![](../images~/hud.png)

The top-left panel in the Game view provides a GUI system that offers tabs to toggle additional information about the state of the ROS communication and visualizations.

The default tabs on the HUD panel includes:

- **Topics**: Contains a list of all ROS topics on which this current session has sent or received a message. The `UI` toggle enables a window that shows the last message sent or received on that topic. The `Viz` toggle enables an in-scene drawing that represents the last message sent or received on that topic. If no `Viz` toggle is enabled, that topic does not have a default visualizer enabled in the Unity scene.
- **Transforms**: Contains [`tf`](http://wiki.ros.org/tf) visualization options, including displaying the axes, links, and labels for each frame.
- **Layout**: Contains options to save and load this visualization configuration. While the visualization components are by default saved via the scene or the prefab, the window layout and visualized message list is saved as a JSON file. By default, this file is saved to a `RosHudLayout.json` file on your machine's [`Application.persistentDataPath`](https://docs.unity3d.com/ScriptReference/Application-persistentDataPath.html) and loaded on each session. In this Layout tab, you can choose to `Export` this JSON file with a custom name to a chosen location on your device, as well as `Import` a layout JSON file to begin using that saved visualization configuration.
<!-- - **Markers**: TODO -->

[TEMP link] The HUD is also designed to be customizable; you may add custom tabs or headers to the HUD. You can write a custom script similar to the [VisualizationLayoutTab](../Runtime/Scripts/VisualizationLayoutTab.cs) to extend the HUD. <!-- - TODO -->

## Using the Inspector

The visualizers for each message type are implemented as Unity MonoBehaviours that are added as components to a GameObject in the scene. This is provided via the `DefaultVisualizationSuite` prefab, or in any custom visualization suite. In the `DefaultVisualizationSuite`, each individual default visualizer can be found by expanding the GameObject in the hierarchy and selecting the GameObject corresponding to the message type's package, e.g. `Geometry` for geometry_msgs.

### Message Topics

Visualizations, by default, are created based on ROS message types. However, you can also directly assign a topic in the visualizer component's Inspector--you can find the `<Type> Default Visualizer` component by expanding the `DefaultVisualizationSuite` GameObject in the Hierarchy and selecting the child object for the package. In the Inspector window, you will see all the default visualizers provided for this package (you may need to scroll down to see all added components).

The **Topic** field can be specifically assigned to customize visualizations for only that topic. This is particularly useful for adding multiple default visualizers of the same ROS message type, customized for different topics.

![](../images~/point_inspector.png)

### TF Topics and Tracking

For messages with stamped headers, there is an option to customize the coordinate frame tracking per visualization. This is provided in the applicable default visualizers via the `TF Tracking Settings`, which contains options for a topic string and a type.

**TF Topic:** It is important to render 3D visualizations in the proper coordinate frame. By default, the `TF Topic` is assigned to `/tf`, but this can be replaced with a different or namespaced TF topic.

**Tracking Type - Exact:** This setting adds the visualization drawing as a child of the `BasicDrawingManager`. The drawing's transform will be modified directly by looking back in time and using the *exact* transform corresponding to the header's timestamp.

**Tracking Type - Track Latest:** This setting places the visualization drawing as a child GameObject corresponding to the proper `frame_id`. The drawing will have a zeroed local position and rotation, and the *frame* GameObject will be transformed based on the *latest* transform information.

**Tracking Type - None:** This setting will set the local position of the drawing to `Vector3.zero` and the local rotation to be `Quaternion.identity`.

### Visualization Settings

The 3D visualizations offer customizations such as `label` and `color` fields, which will modify the drawing in the scene. Visualizations including lines or arrows (e.g. `sensor_msgs/Imu`) provide options for the length and thickness of the arrow as well as the radius around which any curved arrows are drawn. These customizations will be specific to each message type. Please note that changes to these settings will not be saved during runtime, and you will have to exit Play mode to save these modifications.

> Note: Size-related fields are in Unity coordinates, where 1 unit = 1 meter.

### Joy Messages

This package contains preconfigured button maps for the Xbox 360 wired and wireless controllers for Windows and Linux mappings, provided as ScriptableObjects in the [TEMP link] [`Resources/VisualizerSettings`](https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/laurie/JoySettings/com.unity.robotics.visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects) directory.

You can create your own custom mapping for the Joy Default Visualizer by right-clicking in the Project window under `Create > Robotics > Sensor Visualizers > Joy`. Once the file is made, you can click into the asset and manually assign the button or axis index appropriate for your custom controller.

Once the mapping is done, in your Joy Default Visualizer component (e.g. `DefaultVisualizationSuite/Sensor/JoyDefaultVisualizer`), assign the `Settings` field to your newly made button map.

### More on Point Clouds

Similar to the Visualization Settings, point cloud visualizations are highly customizable. Settings for these message visualizers (PointCloud, LaserScan, etc.) will be saved during runtime. For more information on this, you can check out the [TEMP link] [base SettingsBasedVisualizer](../Editor/SettingsBasedVisualizerEditor.cs) class, as well as read more about Unity's [ScriptableObjects](https://docs.unity3d.com/Manual/class-ScriptableObject.html).

The standard settings are provided in ScriptableObjects. Default settings are provided in the [TEMP link] [`Resources/VisualizerSettings`](https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/laurie/JoySettings/com.unity.robotics.visualizations/Runtime/DefaultVisualizers/Sensor/ScriptableObjects) directory, and can be created by right-clicking in the Project window under `Create > Robotics > Sensor Visualizers`. After being created, this configuration can be dragged and dropped into the component's Inspector field `Visualizer settings,` or selected by clicking on the small circle to the right of the field.

![](../images~/pcl2.png)

The settings available will depend on the ROS message type.

**Channel Name**: These settings allow you to choose which channel name corresponds to X, Y, Z, and color channels.

**Size**: Each point is by default a uniform size. This toggle allows you to select a channel that defines the size of each drawn point, e.g. using `intensity` to assign scale based on each point's intensity reading.

**Color**: The color options enable the point clouds to be drawn with configurable colors.

  - The `HSV` option allows you to choose a channel that will be automatically converted to colored points. This can be useful for visualizing individual lasers like a lidar's `ring` channel, for example.

  - The `Combined RGB` option is used for channels that should specifically be parsed into RGB data, e.g. `rgb` channels.

  - The `Separate RGB` is similar to the HSV option, but assigns a different channel to each R, G, and B color channel, which may be used for visualizing X, Y, and Z axes, for example.

**Range**: The min and max value fields configure the ranges for the sliding bar provided. This setting is applied to the respective range field.