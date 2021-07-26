using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class DrawingStampedVisualFactory<TMessageType> : DrawingVisualFactory<TMessageType>,
        IVisualDrawer<TMessageType>, IVisualFactory, IPriority
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
