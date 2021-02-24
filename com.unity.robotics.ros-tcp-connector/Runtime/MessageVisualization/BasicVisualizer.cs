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
        public string Topic;
        public string Label;
        public Color Color;

        [HideInInspector][SerializeField]
        string TopicCopy;

        public int Priority { get; set; }
        public virtual System.Type RegisteredMessageType => typeof(MessageType);

        public virtual void Start()
        {
            if (Topic == "")
                MessageVisualizations.RegisterVisualizer(RegisteredMessageType, this, Priority);
            else
                MessageVisualizations.RegisterVisualizer(Topic, this, Priority);
        }

        public void OnValidate()
        {
            if (Topic != TopicCopy)
            {
                if (Color == Color.clear || Color == Color.black || Color == MessageVisualizations.PickColorForTopic(TopicCopy))
                    Color = MessageVisualizations.PickColorForTopic(Topic);
                if (Label == TopicCopy || Label == "")
                    Label = Topic;
            }

            if (Color.a == 0)
                Color.a = 1;

            TopicCopy = Topic;
        }

        public virtual object CreateDrawing(Message msg, MessageMetadata meta)
        {
            if (!AssertMessageType(msg, meta))
            {
                return null;
            }

            Color colorToUse = this.Color;
            string labelToUse = this.Label;

            if (colorToUse == Color.clear || colorToUse == Color.black)
                colorToUse = MessageVisualizations.PickColorForTopic(meta.topic);

            if (labelToUse == "" || labelToUse == null)
                labelToUse = meta.topic;

            DebugDraw.Drawing drawing = DebugDraw.CreateDrawing();
            Draw((MessageType)msg, meta, colorToUse, labelToUse, drawing);
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
            if (!(RegisteredMessageType.IsInstanceOfType(msg)))
            {
                Debug.LogError($"{this.GetType()}, visualizer for topic \"{meta.topic}\": Can't visualize message type {msg.GetType()}! (expected {RegisteredMessageType}).");
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