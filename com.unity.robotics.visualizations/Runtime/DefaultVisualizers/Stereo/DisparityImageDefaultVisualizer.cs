using RosMessageTypes.Stereo;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public class DisparityImageDefaultVisualizer : TextureVisualizer<DisparityImageMsg>
    {
        static Material s_Material;

        public override Action CreateGUI(DisparityImageMsg message, MessageMetadata meta, Texture2D tex)
        {
            if (s_Material == null)
                s_Material = new Material(Shader.Find("Unlit/DisparityImage"));
            Material material = new Material(s_Material);
            material.SetFloat("_MinDisparity", message.min_disparity);
            material.SetFloat("_MaxDisparity", message.max_disparity);

            return () =>
            {
                message.image.header.GUI();
                GUILayout.Label($"f: {message.f}, T: {message.t}, delta_d:{message.delta_d}");
                GUILayout.Label($"Disparity: [{message.min_disparity}..{message.max_disparity}]");
                GUILayout.Label($"Region of interest: ");
                message.valid_window.GUI();
                tex.GUITexture(material);
            };
        }

        public override Texture2D CreateTexture(DisparityImageMsg message)
        {
            return message.image.ToTexture2D(false);
        }
    }
}
