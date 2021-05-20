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
    public Texture2D tex { get; private set; }
    public bool useCameraInfo = false; // TODO
    string m_CameraInfoTopic;
    MCameraInfo m_CameraInfoMessage;
    bool convertBgr = true;
    bool prevConvert = true;
    bool flipY = true;
    bool prevFlip = true;

    void AssignInfo(MCameraInfo cameraInfo)
    {
        m_CameraInfoMessage = cameraInfo;
    }

    // TODO
    public override void Draw(BasicDrawing drawing, MImage message, MessageMetadata meta)
    {
        // if (m_CameraInfoMessage != null && message.data.Length > 0)
        // {
        //     var worldPoints = m_CameraInfoMessage.GetPixelsInWorld();
        //     message.Draw<FLU>(drawing, color, worldPoints);
        // }
    }

    public override Action CreateGUI(MImage message, MessageMetadata meta, BasicDrawing drawing)
    {
        if (meta.Topic.Length > 0)
        {
            var parse = meta.Topic.Split('/');
            var camInfo = new string[parse.Length - 1];
            Array.Copy(parse, camInfo, parse.Length - 1);
            var topic = $"{String.Join("/", camInfo)}/camera_info";
            if (topic != m_CameraInfoTopic)
            {
                m_CameraInfoTopic = topic;
                ROSConnection.instance.Subscribe<MCameraInfo>(m_CameraInfoTopic, (camInfo) => { AssignInfo(camInfo); });
            }
        }

        if (message.data.Length > 0)
        {
            tex = message.ToTexture2D(convertBgr, flipY, m_CameraInfoMessage);
        }

		return () =>
		{
            message.header.GUI();
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (!message.encoding.Contains("1") && !message.encoding.Contains("mono") && !message.encoding.Contains("bayer"))
                    convertBgr = GUILayout.Toggle(convertBgr, "From BGR");
                flipY = GUILayout.Toggle(flipY, "Flip Y");
                GUILayout.EndHorizontal();

                if (convertBgr != prevConvert || flipY != prevFlip)
                    tex = message.ToTexture2D(convertBgr, flipY, m_CameraInfoMessage);
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
