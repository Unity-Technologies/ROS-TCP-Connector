using System;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class TexturedDrawingVisual<T> : DrawingVisual<T>, IDrawingTextureVisual
        where T : Message
    {
        new TexturedDrawingVisualFactory<T> m_Drawer;

        public TexturedDrawingVisual(T newMessage, MessageMetadata newMeta, TexturedDrawingVisualFactory<T> drawer, Texture2D tex, TFTrackingType tfTrackingType = TFTrackingType.None,
            HeaderMsg headerMsg = null)
            : base(newMessage, newMeta, drawer, tfTrackingType, headerMsg)
        {
            m_Drawer = drawer;
            texture2D = tex;
        }

        public GameObject drawingObject { get; set; }
        public Texture2D texture2D { get; set; }
        public Material shaderMaterial { get; set; }
        public Material material { get; set; }
        public Mesh mesh { get; set; }

        public Texture2D GetTexture()
        {
            if (texture2D == null)
            {
                texture2D = m_Drawer.CreateTexture(message);
            }
            return texture2D;
        }
    }
}
