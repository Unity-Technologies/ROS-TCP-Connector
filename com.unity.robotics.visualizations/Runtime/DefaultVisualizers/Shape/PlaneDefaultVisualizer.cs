using System;
using RosMessageTypes.Shape;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PlaneDefaultVisualizer : DrawingVisualizer<PlaneMsg>
{
    [SerializeField]
    Color m_Color;

    public override void Draw(Drawing3d drawing, PlaneMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta));
    }

    public static void Draw<C>(PlaneMsg message, Drawing3d drawing, Color color, GameObject center = null, float size = 10.0f)
        where C : ICoordinateSpace, new()
    {
        Draw<C>(message, drawing, color, (center != null) ? center.transform.position : Vector3.zero, size);
    }

    public static void Draw<C>(PlaneMsg message, Drawing3d drawing, Color color, Vector3 origin, float size = 10.0f) where C : ICoordinateSpace, new()
    {
        Vector3 normal = new Vector3<C>((float)message.coef[0], (float)message.coef[1], (float)message.coef[2]).toUnity;
        float d = (float)message.coef[3];

        float normalScale = (Vector3.Dot(normal, origin) + d) / normal.sqrMagnitude;
        Vector3 center = origin - normal * normalScale;

        Vector3 forward = (Mathf.Abs(normal.x) > Mathf.Abs(normal.y)) ? Vector3.Cross(normal, Vector3.up).normalized : Vector3.Cross(normal, Vector3.right).normalized;
        Vector3 side = Vector3.Cross(normal, forward).normalized;
        Vector3 diagonalA = (forward + side) * size;
        Vector3 diagonalB = (forward - side) * size;
        drawing.DrawQuad(center - diagonalA, center + diagonalB, center + diagonalA, center - diagonalB, color, true);
    }

    public override Action CreateGUI(PlaneMsg message, MessageMetadata meta)
    {
        return () =>
        {
            GUILayout.Label($"[{message.coef[0]}, {message.coef[1]}, {message.coef[2]}, {message.coef[3]}]");
        };
    }
}
