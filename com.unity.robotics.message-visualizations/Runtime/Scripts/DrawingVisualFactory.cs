using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingVisualFactory<TMessageType> : MonoBehaviour, IVisualDrawer<TMessageType>, IVisualFactory, IPriority
        where TMessageType : Message
    {
        [SerializeField]
        string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        public virtual void Start()
        {
            if (m_Topic == null || m_Topic == "")
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<TMessageType>(this, Priority);
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
            if (!AssertMessageType(message, meta))
                return null;

            return new DrawingVisual<TMessageType>((TMessageType)message, meta, this);
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

        public virtual void Draw(BasicDrawing drawing, TMessageType message, MessageMetadata meta) { }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TMessageType))
            {
                Debug.LogError($"{GetType()}, visualFactory for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TMessageType)}).");
                return false;
            }

            return true;
        }

        public virtual Action CreateGUI(TMessageType message, MessageMetadata meta)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }
    }
}
