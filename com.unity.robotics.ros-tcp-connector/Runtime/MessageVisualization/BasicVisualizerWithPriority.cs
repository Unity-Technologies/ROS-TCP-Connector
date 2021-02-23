using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class BasicVisualizerWithPriority<Msg> : BasicVisualizer<Msg> where Msg:Message
    {
        public int priority;

        public override void Start()
        {
            if (topic == "")
                MessageVisualizations.RegisterVisualizer<Msg>(this, priority);
            else
                MessageVisualizations.RegisterVisualizer(topic, this, priority);
        }
    }
}
