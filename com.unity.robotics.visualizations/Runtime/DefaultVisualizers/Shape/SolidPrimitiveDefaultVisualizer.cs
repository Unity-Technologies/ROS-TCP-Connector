using System;
using RosMessageTypes.Shape;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class SolidPrimitiveDefaultVisualizer : DrawingVisualizer<SolidPrimitiveMsg>
{
    [SerializeField]
    GameObject m_Origin;
    [SerializeField]
    Color m_Color;

    public override void Draw(Drawing3d drawing, SolidPrimitiveMsg message, MessageMetadata meta)
    {
        Draw<FLU>(message, drawing, SelectColor(m_Color, meta), m_Origin);
    }

    public static void Draw<C>(SolidPrimitiveMsg message, Drawing3d drawing, Color color, GameObject origin = null)
    where C : ICoordinateSpace, new()
    {
        Vector3 originPosition = origin != null ? origin.transform.position : Vector3.zero;
        Quaternion originRotation = origin != null ? origin.transform.rotation : Quaternion.identity;
        switch (message.type)
        {
            case SolidPrimitiveMsg.BOX:
                drawing.DrawCuboid(
                    originPosition,
                    new Vector3<C>(
                        (float)message.dimensions[SolidPrimitiveMsg.BOX_X] * 0.5f,
                        (float)message.dimensions[SolidPrimitiveMsg.BOX_Y] * 0.5f,
                        (float)message.dimensions[SolidPrimitiveMsg.BOX_Z] * 0.5f).toUnity,
                    originRotation,
                    color
                );
                break;
            case SolidPrimitiveMsg.SPHERE:
                drawing.DrawSphere(originPosition, color, (float)message.dimensions[SolidPrimitiveMsg.SPHERE_RADIUS]);
                break;
            case SolidPrimitiveMsg.CYLINDER:
                Vector3 cylinderAxis = originRotation * Vector3.up * (float)message.dimensions[SolidPrimitiveMsg.CYLINDER_HEIGHT] * 0.5f;
                drawing.DrawCylinder(originPosition - cylinderAxis, originPosition + cylinderAxis, color, (float)message.dimensions[SolidPrimitiveMsg.CYLINDER_RADIUS]);
                break;
            case SolidPrimitiveMsg.CONE:
                Vector3 coneAxis = originRotation * Vector3.up * (float)message.dimensions[SolidPrimitiveMsg.CONE_HEIGHT] * 0.5f;
                drawing.DrawCone(originPosition - coneAxis, originPosition + coneAxis, color, (float)message.dimensions[SolidPrimitiveMsg.CONE_RADIUS]);
                break;
        }
    }

    public override Action CreateGUI(SolidPrimitiveMsg message, MessageMetadata meta)
    {
        return () =>
        {
            message.GUI();
        };
    }
}
