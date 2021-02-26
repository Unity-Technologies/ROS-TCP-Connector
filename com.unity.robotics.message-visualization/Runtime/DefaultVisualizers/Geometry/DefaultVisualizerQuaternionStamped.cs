using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerQuaternionStamped : BasicVisualizer<MQuaternionStamped>
    {
        public float size = 0.01f;
        public GameObject drawAtPosition;

        public override void Draw(MQuaternionStamped message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            MessageVisualizations.Draw<FLU>(drawing, message.quaternion, drawAtPosition, size);
            drawing.DrawLabel(label, transform.position, color, size);
        }

        public override Action CreateGUI(MQuaternionStamped message, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(message.header);
            MessageVisualizations.GUI(message.quaternion);
        };
    }
}