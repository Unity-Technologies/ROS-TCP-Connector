using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class RosPublish : MonoBehaviour
    {
        [SerializeField]
        string m_Topic;
        [SerializeField]
        string m_RosMessageName;
        [SerializeField]
        string m_FrameID;
        ROSConnection m_Ros;
        VisualizationTrigger m_VisualizationTrigger;
        bool m_Stamped;

        public virtual void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher(m_Topic, m_RosMessageName);
            m_Stamped = m_RosMessageName.Contains("Stamped"); // TODO: Better manage what types need stamped headers
            m_VisualizationTrigger = GetComponent<VisualizationTrigger>();
        }

        public void PublishString(string s)
        {
            m_Ros.Send(m_Topic, new StringMsg(s));
        }

        public void PublishPoint(Vector3 point)
        {
            PointMsg pointMsg = point.To<FLU>();
            if (m_Stamped)
            {
                m_Ros.Send(m_Topic, new PointStampedMsg
                {
                    header = new HeaderMsg(new TimeStamp(Clock.time), m_FrameID),
                    point = pointMsg
                });
            }
            else
            {
                m_Ros.Send(m_Topic, pointMsg);
            }
        }

        public void PublishPose((Vector3, Quaternion) pose)
        {
            var (pos, rot) = pose;
            var poseMsg = new PoseMsg
            {
                position = pos.To<FLU>(),
                orientation = rot.To<FLU>()
            };

            if (m_Stamped)
            {
                m_Ros.Send(m_Topic, new PoseStampedMsg
                {
                    header = new HeaderMsg(new TimeStamp(Clock.time), m_FrameID),
                    pose = poseMsg
                });
            }
            else
            {
                m_Ros.Send(m_Topic, poseMsg);
            }
        }

        public void ResetTrigger()
        {
            m_VisualizationTrigger.Reset();
        }
    }
}
