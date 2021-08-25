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
        [SerializeField]
        string m_TFTopic = "/tf";
        public string TFTopic { get => m_TFTopic; set => m_TFTopic = value; }

        void Start()
        {
            transform.parent = TFSystem.GetOrCreateInstance().GetTransformObject(m_FrameID, m_TFTopic).transform;
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
        }
    }
}
