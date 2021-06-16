using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicVisualFactory<TargetMessageType> : MonoBehaviour, IVisualFactory, IPriority
        where TargetMessageType : Message
    {
        [SerializeField]
        string m_Topic;

        public virtual void Start()
        {
            if (m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<TargetMessageType>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }

        public int Priority { get; set; }

        public bool CanShowDrawing => true;

        public IVisual CreateVisual(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta)) return null;
            return new BasicVisual<TargetMessageType>((TargetMessageType)message, meta, this);
        }

        public BasicDrawing CreateDrawing(Message message, MessageMetadata meta, object oldDrawing)
        {
            if (!AssertMessageType(message, meta)) return null;

            BasicDrawing drawing;
            if (oldDrawing != null)
            {
                drawing = (BasicDrawing)oldDrawing;
                drawing.Clear();
            }
            else
            {
                drawing = BasicDrawingManager.CreateDrawing();
            }

            Draw(drawing, (TargetMessageType)message, meta);
            return drawing;
        }

        public static Color SelectColor(Color userColor, MessageMetadata meta)
        {
            if (userColor.r == 0 && userColor.g == 0 && userColor.b == 0)
                return MessageVisualizations.PickColorForTopic(meta.Topic);

            if (userColor.a == 0)
                return new Color(userColor.r, userColor.g, userColor.b, 1);

            return userColor;
        }

        public static string SelectLabel(string userLabel, MessageMetadata meta)
        {
            if (userLabel == null || userLabel == "")
                return meta.Topic;

            return userLabel;
        }

        public virtual void Draw(BasicDrawing drawing, TargetMessageType message, MessageMetadata meta) { }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TargetMessageType))
            {
                Debug.LogError($"{GetType()}, visualFactory for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TargetMessageType)}).");
                return false;
            }

            return true;
        }

        public virtual Action CreateGUI(TargetMessageType message, MessageMetadata meta)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }
    }
}
