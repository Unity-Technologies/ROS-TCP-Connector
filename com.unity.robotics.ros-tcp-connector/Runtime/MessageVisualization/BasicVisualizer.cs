using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicVisualizer<MessageType> : MonoBehaviour, IVisualizer, IPriority
        where MessageType : Message
    {
        public string topic;
        public string label;
        public Color color;

        [HideInInspector][SerializeField]
        string topicCopy;

        public int priority { get; set; }
        public virtual System.Type registeredMessageType => typeof(MessageType);

        public virtual void Start()
        {
            if (topic == "")
                MessageVisualizations.RegisterVisualizer(registeredMessageType, this, priority);
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

            Color colorToUse = this.color;
            string labelToUse = this.label;

            if (colorToUse == Color.clear || colorToUse == Color.black)
                colorToUse = MessageVisualizations.PickColorForTopic(meta.topic);

            if (labelToUse == "" || labelToUse == null)
                labelToUse = meta.topic;

            DebugDraw.Drawing drawing = DebugDraw.CreateDrawing();
            Draw((MessageType)message, meta, colorToUse, labelToUse, drawing);
            return drawing;
        }

        public virtual void Draw(MessageType msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
        }

        public void DeleteDrawing(object drawing)
        {
            ((DebugDraw.Drawing)drawing).Destroy();
        }

        public virtual System.Action CreateGUI(Message msg, MessageMetadata meta, object drawing)
        {
            if (!AssertMessageType(msg, meta))
            {
                return MessageVisualizations.CreateDefaultGUI(msg, meta);
            }

            return CreateGUI((MessageType)msg, meta, (DebugDraw.Drawing)drawing);
        }

        public bool AssertMessageType(Message msg, MessageMetadata meta)
        {
            if (!(registeredMessageType.IsInstanceOfType(msg)))
            {
                Debug.LogError($"{this.GetType()}, visualizer for topic \"{meta.topic}\": Can't visualize message type {msg.GetType()}! (expected {registeredMessageType}).");
                return false;
            }
            return true;
        }

        public virtual System.Action CreateGUI(MessageType msg, MessageMetadata meta, DebugDraw.Drawing drawing)
        {
            return MessageVisualizations.CreateDefaultGUI(msg, meta);
        }
    }
}