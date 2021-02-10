using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    [System.AttributeUsage(System.AttributeTargets.Class, AllowMultiple = true)]
    public class AutoRegisteredVisualizerAttribute : System.Attribute
    {
        public readonly string topic;
        public readonly int priority;

        public AutoRegisteredVisualizerAttribute(string topic = null)
        {
            this.topic = topic;
        }

        public AutoRegisteredVisualizerAttribute(int priority)
        {
            this.priority = priority;
        }
    }
}