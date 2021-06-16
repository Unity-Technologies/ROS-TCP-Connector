using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMagneticField : DrawingVisualFactory<MMagneticField>
{
    [SerializeField]
    Color m_Color;
    bool m_ViewCovariance;

    public override void Draw(BasicDrawing drawing, MMagneticField message, MessageMetadata meta)
    {
        message.Draw<FLU>(drawing, SelectColor(m_Color, meta));
    }

    public override Action CreateGUI(MMagneticField message, MessageMetadata meta) => () =>
    {
        message.header.GUI();
        message.magnetic_field.GUI("Magnetic field (Tesla)");
        MessageVisualizations.GUIGrid(message.magnetic_field_covariance, 3, "Covariance", ref m_ViewCovariance);
    };
}
