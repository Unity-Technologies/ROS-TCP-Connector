using System;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class Interaction : MonoBehaviour
    {
        protected RosPublish m_RosPublish;
        protected ClickState m_State;

        public virtual void Start()
        {
            m_RosPublish = GetComponent<RosPublish>();
        }

        public void OnNext(ClickState state)
        {
            m_State = state;
        }
    }
}
