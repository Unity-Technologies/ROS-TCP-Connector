using System;
using RosSharp.Urdf;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicTextureVisualizer<TargetMessageType> : BasicHudOnlyVisualizer<TargetMessageType>
        where TargetMessageType : Message
    {
        protected ITextureMessageVisualization m_Visualization;
        
        public override IMessageVisualization CreateVisualization(Message message, MessageMetadata meta, bool withGui, bool withDrawing)
        {
            var action = CreateGUI(message, meta, null);
            var vis = new BasicTextureVisualization(message, meta, action, null);
            VisualizationRegistry.RegisterTopicVisualization(meta.Topic, vis);
            return vis;
        }
    }
}
