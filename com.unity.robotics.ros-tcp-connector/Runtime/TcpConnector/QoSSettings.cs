using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public enum QoSDurabilityPolicy
    {
        Default = 0,
        TransientLocal = 1,
        Volatile = 2,
        Unknown = 3
    }

    public enum QoSHistoryPolicy
    {
        Default = 0,
        KeepLast = 1,
        KeepAll = 2,
        Unknown = 3
    }

    public enum QoSLivelinessPolicy
    {
        Default = 0,
        Automatic = 1,
        ManualByTopic = 3,
        Unknown = 4
    }

    public enum QoSReliability
    {
        Default = 0,
        BestEffort = 1,
        Reliable = 2,
        Unknown = 3
    }

    public class QoSSettings
    {
        public float deadline = -1;
        public int depth = 10;
        public QoSDurabilityPolicy durability = QoSDurabilityPolicy.Volatile;
        public QoSHistoryPolicy history = QoSHistoryPolicy.KeepLast;
        public float lifespan = -1;
        public QoSLivelinessPolicy liveliness = QoSLivelinessPolicy.Default;
        public float liveliness_lease_duration = -1;
        public QoSReliability reliability = QoSReliability.Reliable;
    }
}
