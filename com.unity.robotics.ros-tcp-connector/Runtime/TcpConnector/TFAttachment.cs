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

        void Start()
        {
            transform.parent = TFSystem.GetOrCreateInstance().GetTransformObject(m_FrameID).transform;
            transform.localPosition = Vector3.zero;
            transform.localRotation = Quaternion.identity;
        }
    }
}
