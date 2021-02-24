using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IPriority
    {
        int priority { get; set; }
    }

    public class PrioritySetter : MonoBehaviour
    {
        public int Priority;

        private void Awake()
        {
            foreach (IPriority p in GetComponents<IPriority>())
                p.priority = Priority;
        }
    }
}