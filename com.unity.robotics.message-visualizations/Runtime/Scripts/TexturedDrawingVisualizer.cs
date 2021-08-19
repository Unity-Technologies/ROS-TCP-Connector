using System;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class TexturedDrawingVisualizer<T> : DrawingVisualizer<T>
        where T : Message
    {
        public override IVisual CreateVisual()
        {
            return new TexturedDrawingVisual(this);
        }

        public abstract Texture2D CreateTexture(T message, MessageMetadata meta);

        public class TexturedDrawingVisual : DrawingVisualizer<T>.DrawingVisual, IDrawingTextureVisual
        {
            TexturedDrawingVisualizer<T> m_Drawer;

            public TexturedDrawingVisual(TexturedDrawingVisualizer<T> drawer)
                : base(drawer)
            {
                m_Drawer = drawer;
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
                    texture2D = m_Drawer.CreateTexture(message, meta);
                }
                return texture2D;
            }
        }
    }
}
