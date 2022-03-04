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
        public float deadline;
        public int depth;
        public QoSDurabilityPolicy durability;
        public QoSHistoryPolicy history;
        public float lifespan;
        public QoSLivelinessPolicy liveliness;
        public float lifeliness_lease_duration;
        public QoSReliability reliability;
    }
}
