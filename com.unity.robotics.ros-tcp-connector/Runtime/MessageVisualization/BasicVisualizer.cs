using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicVisualizer<Msg> : MonoBehaviour, IVisualizer where Msg : Message
    {
        public string topic;
        public string label;
        public Color color;

        [HideInInspector][SerializeField]
        string topicCopy;

        public virtual void Start()
        {
            if (topic == "")
                MessageVisualizations.RegisterVisualizer<Msg>(this);
            else
                MessageVisualizations.RegisterVisualizer(topic, this);
        }

        public void OnValidate()
        {
            if(topic != topicCopy)
            {
                if (color == Color.clear || color == Color.black || color == MessageVisualizations.PickColorForTopic(topicCopy))
                    color = MessageVisualizations.PickColorForTopic(topic);
                if (label == topicCopy || label == "")
                    label = topic;
            }

            if (color.a == 0)
                color.a = 1;

            topicCopy = topic;
        }

        public object CreateDrawing(Message msg, MessageMetadata meta)
        {
            if(!(msg is Msg))
            {
                Debug.LogError("Visualizer on "+meta.topic+": Invalid message type " + msg.GetType() + ", expected " + typeof(Msg));
                return null;
            }

            Color colorToUse = this.color;
            string labelToUse = this.label;
            
            if (colorToUse == Color.clear || colorToUse == Color.black)
                colorToUse = MessageVisualizations.PickColorForTopic(meta.topic);
            
            if (labelToUse == "" || labelToUse == null)
                labelToUse = meta.topic;

            DebugDraw.Drawing drawing = DebugDraw.CreateDrawing();
            Draw((Msg)msg, meta, colorToUse, labelToUse, drawing);
            return drawing;
        }

        public virtual void Draw(Msg msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
        }

        public void DeleteDrawing(object drawing)
        {
            ((DebugDraw.Drawing)drawing).Destroy();
        }

        public System.Action CreateGUI(Message msg, MessageMetadata meta, object drawing)
        {
            if (!(msg is Msg))
            {
                Debug.LogError("Visualizer on " + meta.topic + ": Invalid message type " + msg.GetType() + ", expected " + typeof(Msg));
                return MessageVisualizations.CreateDefaultGUI(msg, meta);
            }

            return CreateGUI((Msg)msg, meta, (DebugDraw.Drawing)drawing);
        }

        public virtual System.Action CreateGUI(Msg msg, MessageMetadata meta, DebugDraw.Drawing drawing)
        {
            return MessageVisualizations.CreateDefaultGUI(msg, meta);
        }
    }
}