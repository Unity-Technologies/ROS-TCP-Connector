using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMagneticField : BasicVisualizer<MMagneticField>
{
    public override void Draw(BasicDrawing drawing, MMagneticField message, MessageMetadata meta, Color color, string label)
    {
        message.Draw<FLU>(drawing, color);
    }

    public override Action CreateGUI(MMagneticField message, MessageMetadata meta, BasicDrawing drawing) => () =>
    {
        message.header.GUI();
        message.magnetic_field.GUI("Magnetic field (Tesla)");
        MessageVisualizations.GUIGrid(message.magnetic_field_covariance, 3, "Covariance");
    };
}
