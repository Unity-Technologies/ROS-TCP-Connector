using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class RangeDefaultVisualizer : DrawingVisualizer<RangeMsg>
{
    [SerializeField]
    Color m_Color;
    [SerializeField]
    TFTrackingSettings m_TFTrackingSettings;

    public override void Draw(Drawing3d drawing, RangeMsg message, MessageMetadata meta)
    {
        drawing.SetTFTrackingSettings(m_TFTrackingSettings, message.header);
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta));
    }

    public static void Draw<C>(RangeMsg message, Drawing3d drawing, Color color, float size = 0.1f, bool drawUnityAxes = false)
        where C : ICoordinateSpace, new()
    {
        var asin = Mathf.Asin(message.field_of_view);
        var acos = Mathf.Acos(message.field_of_view);
        Color col = Color.HSVToRGB(Mathf.InverseLerp(message.min_range, message.max_range, message.range), 1, 1);
        Vector3 end = new Vector3<C>(message.range * acos, 0, message.range * asin).toUnity;

        drawing.DrawCone(end, Vector3.zero, col, Mathf.Rad2Deg * message.field_of_view / 2);
    }

    public override Action CreateGUI(RangeMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.header.GUI();
            GUILayout.Label($"Radiation type: {(Range_RadiationType_Constants)message.radiation_type}\nFOV: {message.field_of_view} (rad)\nMin range: {message.min_range} (m)\nMax range: {message.max_range} (m)\nRange: {message.range} (m)");
        };
    }
}
