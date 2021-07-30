using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class StampedDrawingVisualFactory<TMessageType> : DrawingVisualFactory<TMessageType>
        where TMessageType : Message
    {
        [SerializeField]
        protected TFTrackingType m_TFTrackingType = TFTrackingType.Exact;

        public TFTrackingType TFTrackingType
        {
            get => m_TFTrackingType;
            set => m_TFTrackingType = value;
        }
    }
}
