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
        public string topic;
        public T message;

        public MessageVisualizerDrawing()
        {
        }

        public void Begin(string topic, T msg)
        {
            drawing = DebugDraw.CreateDrawing();
            this.topic = topic;
            message = (T)msg;
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