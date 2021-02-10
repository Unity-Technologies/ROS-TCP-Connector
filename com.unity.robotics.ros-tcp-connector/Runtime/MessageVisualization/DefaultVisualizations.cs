using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using MPoint = RosMessageTypes.Geometry.Point;
using MPoint32 = RosMessageTypes.Geometry.Point32;
using MVector3 = RosMessageTypes.Geometry.Vector3;
using MTransform = RosMessageTypes.Geometry.Transform;
using MQuaternion = RosMessageTypes.Geometry.Quaternion;
using RosMessageTypes;

[AutoRegisteredVisualizer(-1)]
public class DefaultVisualizer_Point : MessageVisualizerDrawing<MPoint>
{
    public override void DrawVisual()
    {
        MessageVisualizations.Draw<FLU>(drawing, message, Color.red, topic);
    }

    public override void OnGUI()
    {
        MessageVisualizations.GUI(topic, message);
    }
}


[AutoRegisteredVisualizer(-1)]
public class DefaultVisualizer_Point32 : MessageVisualizerDrawing<MPoint32>
{
    public override void DrawVisual()
    {
        MessageVisualizations.Draw<FLU>(drawing, message, Color.red, topic);
    }

    public override void OnGUI()
    {
        MessageVisualizations.GUI(topic, message);
    }
}

[AutoRegisteredVisualizer(-1)]
public class DefaultVisualizer_Vector3 : MessageVisualizerDrawing<MVector3>
{
    public override void DrawVisual()
    {
        MessageVisualizations.Draw<FLU>(drawing, message, Color.red, topic);
    }

    public override void OnGUI()
    {
        MessageVisualizations.GUI(topic, message);
    }
}

[AutoRegisteredVisualizer(-1)]
public class DefaultVisualizer_Transform : MessageVisualizerDrawing<MTransform>
{
    public override void DrawVisual()
    {
        MessageVisualizations.Draw<FLU>(drawing, message);
    }

    public override void OnGUI()
    {
        MessageVisualizations.GUI(topic, message);
    }
}

[AutoRegisteredVisualizer(-1)]
public class DefaultVisualizer_Quaternion : IMessageVisualizer<MQuaternion>
{
    MQuaternion msg;
    string topic;
    public void Begin(MQuaternion msg, MessageMetadata meta)
    {
        this.msg = msg;
        this.topic = meta.topic;
    }

    public void End()
    {
    }

    public void OnGUI()
    {
        MessageVisualizations.GUI(topic, msg);
    }
}