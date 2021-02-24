using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicVisualizer<TargetMessageType> : MonoBehaviour, IVisualizer, IPriority
        where TargetMessageType : Message
    {
        public string topic;
        public string label;
        public Color color;

        [HideInInspector][SerializeField]
        string topicCopy;

        public int priority { get; set; }

        public virtual void Start()
        {
            if (topic == "")
                MessageVisualizations.RegisterVisualizer<TargetMessageType>(this, priority);
            else
                MessageVisualizations.RegisterVisualizer(topic, this, priority);
        }

        public void OnValidate()
        {
            if (topic != topicCopy)
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

        public virtual object CreateDrawing(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta))
            {
                return null;
            }

            DebugDraw.Drawing drawing = DebugDraw.CreateDrawing();
            Draw((TargetMessageType)message, meta, SelectColor(meta), SelectLabel(meta), drawing);
            return drawing;
        }

        public Color SelectColor(MessageMetadata meta)
        {
            if (color != Color.clear && color != Color.black)
                return color;
            else
                return MessageVisualizations.PickColorForTopic(meta.topic);
        }

        public string SelectLabel(MessageMetadata meta)
        {
            if (label != "" && label != null)
                return label;
            else
                return meta.topic;
        }

        public virtual void Draw(TargetMessageType message, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
        }

        public void DeleteDrawing(object drawing)
        {
            ((DebugDraw.Drawing)drawing).Destroy();
        }

        public virtual System.Action CreateGUI(Message message, MessageMetadata meta, object drawing)
        {
            if (!AssertMessageType(message, meta))
            {
                return MessageVisualizations.CreateDefaultGUI(message, meta);
            }

            return CreateGUI((TargetMessageType)message, meta, (DebugDraw.Drawing)drawing);
        }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TargetMessageType))
            {
                Debug.LogError($"{this.GetType()}, visualizer for topic \"{meta.topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TargetMessageType)}).");
                return false;
            }
            return true;
        }

        public virtual System.Action CreateGUI(TargetMessageType message, MessageMetadata meta, DebugDraw.Drawing drawing)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }
    }
}