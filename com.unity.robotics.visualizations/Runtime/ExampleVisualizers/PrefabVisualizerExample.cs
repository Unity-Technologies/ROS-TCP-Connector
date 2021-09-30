using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector;

// A simple visualizer that places a (user configured) prefab to show the position and
// orientation of a Pose message
public class PrefabVisualizerExample : BaseVisualFactory<PoseMsg>
{
    // this setting will appear as a configurable parameter in the Unity editor.
    public GameObject prefab;

    // The BaseVisualFactory's job is just to create visuals for topics as appropriate.
    protected override IVisual CreateVisual(string topic)
    {
        return new PrefabVisual(topic, prefab);
    }

    // The job of the visual itself is to subscribe to a topic, and draw
    // representations of the messages it receives.
    class PrefabVisual : IVisual
    {
        GameObject m_Prefab;
        GameObject m_PrefabInstance;
        PoseMsg m_LastMessage;
        bool m_IsDrawingEnabled;
        public bool IsDrawingEnabled => m_IsDrawingEnabled;

        public PrefabVisual(string topic, GameObject prefab)
        {
            m_Prefab = prefab;

            ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>(topic, AddMessage);
        }

        void AddMessage(PoseMsg message)
        {
            m_LastMessage = message;

            if (m_IsDrawingEnabled)
                Redraw();
        }

        public void SetDrawingEnabled(bool enabled)
        {
            m_IsDrawingEnabled = enabled;

            if (enabled)
                Redraw();
            else
                GameObject.Destroy(m_PrefabInstance);
        }

        public void Redraw()
        {
            if (m_LastMessage == null)
            {
                return;
            }

            GameObject.Destroy(m_PrefabInstance);
            m_PrefabInstance = GameObject.Instantiate(m_Prefab);
            m_PrefabInstance.transform.position = m_LastMessage.position.From<FLU>();
            m_PrefabInstance.transform.rotation = m_LastMessage.orientation.From<FLU>();
        }

        public void OnGUI()
        {
            // Draw the default GUI for a Pose message.
            m_LastMessage.GUI();
        }
    }

    // Indicates that this visualizer should have a "3d" drawing checkbox in the topics list
    public override bool CanShowDrawing => true;
}
