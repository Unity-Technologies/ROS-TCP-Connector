using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerRange : DrawingVisualFactory<RangeMsg>
{
    [SerializeField]
    Color m_Color;

    public override void Draw(BasicDrawing drawing, RangeMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta));
    }

    public static void Draw<C>(RangeMsg message, BasicDrawing drawing, Color color, float size = 0.1f, bool drawUnityAxes = false)
    where C : ICoordinateSpace, new()
    {
        TFFrame frame = TFSystem.instance.GetTransform(message.header);

        var s = Mathf.Asin(message.field_of_view);
        var c = Mathf.Acos(message.field_of_view);
        Color col = Color.HSVToRGB(Mathf.InverseLerp(message.min_range, message.max_range, message.range), 1, 1);

        Vector3 end = new Vector3(message.range * c, 0, message.range * s);
        Matrix4x4 matrix = Matrix4x4.TRS(Vector3.zero, frame.rotation, Vector3.one);
        end = matrix.MultiplyPoint(end);

        drawing.DrawCone(frame.translation + end, frame.translation, col, Mathf.Rad2Deg * message.field_of_view / 2);
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
