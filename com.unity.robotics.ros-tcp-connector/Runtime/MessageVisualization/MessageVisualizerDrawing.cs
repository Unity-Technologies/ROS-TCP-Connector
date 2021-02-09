using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public class MessageVisualizerDrawing<T> : IMessageVisualizer<T> where T : Message
    {
        public DebugDraw.Drawing drawing;
        public T message;
        public MessageMetadata meta;
        public string topic => meta.topic;

        public MessageVisualizerDrawing()
        {
        }

        public void Begin(T message, MessageMetadata meta)
        {
            drawing = DebugDraw.CreateDrawing();
            this.message = (T)message;
            this.meta = meta;
            DrawVisual();
        }

        public virtual void DrawVisual()
        {

        }

        public virtual void OnGUI()
        {
            GUILayout.Label(message.ToString());
        }

        public virtual void End()
        {
            drawing.Destroy();
        }
    }
}