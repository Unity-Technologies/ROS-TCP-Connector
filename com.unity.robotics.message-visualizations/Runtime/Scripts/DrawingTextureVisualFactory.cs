using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingTextureVisualFactory<T> : DrawingStampedVisualFactory<T>, IVisualFactory, IPriority
        where T : Message
    {
        public override IVisual CreateVisual(Message message, MessageMetadata meta)
        {
            if (!AssertMessageType(message, meta))
            {
                return null;
            }
            return new DrawingTextureVisual<T>((T)message, meta, this, CreateTexture((T)message), m_TFTrackingType);
        }

        public abstract Texture2D CreateTexture(T message);
    }
}
