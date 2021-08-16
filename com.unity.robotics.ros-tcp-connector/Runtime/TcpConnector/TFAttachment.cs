using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class TFAttachment : MonoBehaviour
    {
        [SerializeField]
        string m_FrameID;
        public string FrameID { get => m_FrameID; set => m_FrameID = value; }
        ROSConnection m_ROS = new ROSConnection();
        void Start()
        {
            transform.parent = TFSystem.GetOrCreateInstance(m_ROS.TFTopics).GetTransformObject(m_FrameID).transform;
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
        }
    }
}
