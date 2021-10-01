using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Geometry;

public class PointVisualizerExample : DrawingVisualizer<PointMsg>
{
    // these settings will appear as configurable parameters in the Unity editor.
    public float m_Size = 0.1f;
    public Color m_Color;
    public string m_Label;

    public override void Draw(Drawing3d drawing, PointMsg msg, MessageMetadata meta)
    {
        // If the user doesn't specify a color, SelectColor helpfully picks one
        // based on the message topic.
        Color finalColor = VisualizationUtils.SelectColor(m_Color, meta);

        // Most of the default visualizers offer static drawing functions
        // so that your own visualizers can easily send work to them.
        PointDefaultVisualizer.Draw<FLU>(msg, drawing, finalColor, m_Size);

        // You can also directly use the drawing functions provided by the Drawing class
        drawing.DrawLabel(m_Label, msg.From<FLU>(), finalColor, m_Size);
    }

    public override System.Action CreateGUI(PointMsg msg, MessageMetadata meta)
    {
        // this code runs each time a new message is received.
        // If you want to preprocess the message or declare any state variables for
        // the GUI to use, you can do that here.
        string text = $"[{msg.x}, {msg.y}, {msg.z}]";

        return () =>
        {
            // this code runs once per UI event, like a normal Unity OnGUI function.
            GUILayout.Label(text);
        };
    }
}
