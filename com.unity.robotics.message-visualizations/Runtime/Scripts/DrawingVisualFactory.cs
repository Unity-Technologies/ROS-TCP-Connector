using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DrawingVisualFactory<TargetMessageType> : MonoBehaviour, IVisualFactory, IPriority
        where TargetMessageType : Message
    {
        [SerializeField]
        string m_Topic;

        public int Priority { get; set; }
        public bool CanShowDrawing => true;
        
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
        
        public IVisual CreateVisual(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta)) return null;
            return new DrawingVisual<TargetMessageType>((TargetMessageType)message, meta, this);
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
        
        public virtual void Draw(BasicDrawing drawing, TargetMessageType message, MessageMetadata meta) { }

        bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TargetMessageType))
            {
                Debug.LogError($"{GetType()}, visualFactory for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TargetMessageType)}).");
                return false;
            }

            return true;
        }
    }
}
