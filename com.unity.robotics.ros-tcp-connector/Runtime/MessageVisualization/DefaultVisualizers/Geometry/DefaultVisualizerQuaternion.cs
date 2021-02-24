using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using MQuaternion = RosMessageTypes.Geometry.Quaternion;

namespace Unity.Robotics.MessageVisualizers
{
    public class DefaultVisualizerQuaternion : BasicVisualizer<MQuaternion>
    {
        public float size = 0.1f;
        public GameObject drawAtPosition;

        public override void Draw(MQuaternion msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
            Vector3 position = drawAtPosition != null ? drawAtPosition.transform.position : transform.position;
            MessageVisualizations.Draw<FLU>(drawing, msg, position, size);
            drawing.DrawLabel(label, transform.position, color, size);
        }

        public override System.Action CreateGUI(MQuaternion msg, MessageMetadata meta, DebugDraw.Drawing drawing) => () =>
        {
            MessageVisualizations.GUI(Label, msg);
        };
    }
}