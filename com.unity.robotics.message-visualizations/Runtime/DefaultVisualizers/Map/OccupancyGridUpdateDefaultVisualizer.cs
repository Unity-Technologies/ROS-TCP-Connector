using System;
using RosMessageTypes.Map;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class OccupancyGridUpdateDefaultVisualizer : MonoBehaviour, IVisualFactory
{
    [SerializeField]
    string m_OccupancyGridTopic;

    public bool CanShowDrawing => true;

    public IVisual CreateVisual()
    {
        return new OccupancyGridUpdateVisual(this);
    }

    class OccupancyGridUpdateVisual : IVisual
    {
        OccupancyGridDefaultVisualizer.OccupancyGridVisual m_BaseVisual;
        OccupancyGridUpdateDefaultVisualizer m_Settings;

        OccupancyGridUpdateMsg m_Message;
        MessageMetadata m_Meta;

        public OccupancyGridUpdateVisual(OccupancyGridUpdateDefaultVisualizer settings)
        {
            m_Settings = settings;
        }

        public void AddMessage(Message message, MessageMetadata meta)
        {
            if (!MessageVisualizationUtils.AssertMessageType<OccupancyGridUpdateMsg>(message, meta))
                return;

            m_Message = (OccupancyGridUpdateMsg)message;
            m_Meta = meta;
        }

        public void CreateDrawing()
        {
            if (m_BaseVisual == null)
                m_BaseVisual = (OccupancyGridDefaultVisualizer.OccupancyGridVisual)MessageVisualizationUtils.GetVisual(m_Settings.m_OccupancyGridTopic);

            if (m_BaseVisual != null)
                m_BaseVisual.AddUpdate(m_Message);
        }

        public void DeleteDrawing()
        {
        }

        public void OnGUI()
        {
            m_Message.header.GUI();
            GUILayout.Label($"(x, y): ({m_Message.x}, {m_Message.y})");
            GUILayout.Label($"Width x height: {m_Message.width} x {m_Message.height}");
        }
    }
}
