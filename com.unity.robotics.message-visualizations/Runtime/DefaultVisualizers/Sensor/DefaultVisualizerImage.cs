using RosMessageTypes.Sensor;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class DefaultVisualizerImage : BasicVisualizer<MImage>
{
    public static Texture2D tex { get; private set; }
    bool convertBgr = true;
    bool prevConvert = true;
    bool flipY = true;
    bool prevFlip = true;

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        if (message.data.Length > 0)
        {
            tex = message.ToTexture2D(convertBgr, flipY);
        }

		return () =>
		{
            message.header.GUI();
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (!message.encoding.Contains("1") && !message.encoding.Contains("mono") && !message.encoding.Contains("bayer"))
                {
                    convertBgr = GUILayout.Toggle(convertBgr, "From BGR");
                }
                flipY = GUILayout.Toggle(flipY, "Flip Y");
                GUILayout.EndHorizontal();

                if (convertBgr != prevConvert || flipY != prevFlip)
                {
                    tex = message.ToTexture2D(convertBgr, flipY);
                }
            }

            // TODO: Rescale/recenter image based on window height/width
            if (tex != null)
            {
                var origRatio = (float)tex.width / (float)tex.height;
                UnityEngine.GUI.Box(GUILayoutUtility.GetAspectRect(origRatio), tex);
            }
            GUILayout.Label($"Height x Width: {message.height}x{message.width}\nEncoding: {message.encoding}");

            prevConvert = convertBgr;
            prevFlip = flipY;
		};
    }
}
