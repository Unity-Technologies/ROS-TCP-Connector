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
        [SerializeField]
        string m_Topic;
        [SerializeField]
        string m_Label;
        [SerializeField]
        Color m_Color;

        public int Priority { get; set; }

        public virtual void Start()
        {
            if (m_Topic == "")
                VisualizationRegister.RegisterVisualizer<TargetMessageType>(this, Priority);
            else
                VisualizationRegister.RegisterVisualizer(m_Topic, this, Priority);
        }

        public void OnValidate()
        {
            if (m_Color.a == 0)
                m_Color.a = 1;
        }

        public virtual object CreateDrawing(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta))
            {
                return null;
            }

            DebugDraw.Drawing drawing = DebugDraw.CreateDrawing();
            Draw(drawing, (TargetMessageType)message, meta, SelectColor(meta), SelectLabel(meta));
            return drawing;
        }

        public Color SelectColor(MessageMetadata meta)
        {
            if (m_Color != Color.clear && m_Color != Color.black)
                return m_Color;
            else
                return MessageVisualizations.PickColorForTopic(meta.Topic);
        }

        public string SelectLabel(MessageMetadata meta)
        {
            if (m_Label != "" && m_Label != null)
                return m_Label;
            else
                return meta.Topic;
        }

        public virtual void Draw(DebugDraw.Drawing drawing, TargetMessageType message, MessageMetadata meta, Color color, string label)
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
                Debug.LogError($"{this.GetType()}, visualizer for topic \"{meta.Topic}\": Can't visualize message type {message.GetType()}! (expected {typeof(TargetMessageType)}).");
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