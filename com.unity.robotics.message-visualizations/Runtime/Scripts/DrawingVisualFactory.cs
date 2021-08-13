using RosMessageTypes.Std;
using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingVisualFactory<T> : MonoBehaviour, IVisualDrawer<T>, IVisualFactory, IPriority
        where T : Message
    {
        [SerializeField]
        string m_Topic;
        public string Topic { get => m_Topic; set => m_Topic = value; }

        public virtual void Start()
        {
            if (string.IsNullOrEmpty(m_Topic))
            {
                VisualFactoryRegistry.RegisterTypeVisualizer<T>(this, Priority);
            }
            else
            {
                VisualFactoryRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
            }
        }

        public int Priority { get; set; }

        public bool CanShowDrawing => true;

        public virtual IVisual CreateVisual()
        {
            return new DrawingVisual<DrawingVisualFactory<T>, T>(this);
        }

        public virtual HeaderMsg GetHeader(T message)
        {
            return null;
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
            if (string.IsNullOrEmpty(userLabel))
                return meta.Topic;

            return userLabel;
        }

        public virtual void Draw(BasicDrawing drawing, T message, MessageMetadata meta) { }

        public virtual Action CreateGUI(T message, MessageMetadata meta)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }

        public virtual HeaderMsg GetHeader(Message message)
        {
            return null;
        }
    }
}
