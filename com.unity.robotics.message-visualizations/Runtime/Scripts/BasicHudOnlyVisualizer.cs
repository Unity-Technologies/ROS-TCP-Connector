using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicHudOnlyVisualizer<TargetMessageType> : MonoBehaviour, IVisualizer, IPriority
        where TargetMessageType : Message
    {
        [SerializeField]
        string m_Topic;

        public int Priority { get; set; }

        public bool CanShowDrawing => false;

        public virtual void Start()
        {
            if (m_Topic == "")
                VisualizationRegistry.RegisterTypeVisualizer<TargetMessageType>(this, Priority);
            else
                VisualizationRegistry.RegisterTopicVisualizer(m_Topic, this, Priority);
        }

        public IMessageVisualization CreateVisualization(Message message, MessageMetadata meta, bool withGui, bool withDrawing)
        {
            // TODO
            Debug.Log("calling basichudonlyvisulizer createvisualization");
            return null;
        }

        public object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing)
        {
            return null;
        }

        public void DeleteDrawing(object drawing)
        {
        }

        public System.Action CreateGUI(Message message, MessageMetadata meta, object drawing)
        {
            if (!AssertMessageType(message, meta))
            {
                return MessageVisualizations.CreateDefaultGUI(message, meta);
            }

            return CreateGUI((TargetMessageType)message, meta, (BasicDrawing)drawing);
        }

        public bool AssertMessageType(Message message, MessageMetadata meta)
        {
            if (!(message is TargetMessageType))
            {
                Debug.LogError($"{this.GetType()}, visualizer for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TargetMessageType)}).");
                return false;
            }
            return true;
        }

        public virtual System.Action CreateGUI(TargetMessageType message, MessageMetadata meta, BasicDrawing drawing)
        {
            return MessageVisualizations.CreateDefaultGUI(message, meta);
        }
    }
}