using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Tf2;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.TestTools;

namespace UnitTests
{
    public class TFSystemTests
    {
        const string simple_frame_id = "test_frame_id";
        const string simple_frame_id2 = "test_frame_id2";
        const string parent_frame_id = "parent_frame_id";
        const string composite_frame_id = parent_frame_id + "/" + simple_frame_id;
        const string composite_frame_id2 = parent_frame_id + "/" + simple_frame_id2;

        HeaderMsg MakeHeaderMsg(TimeMsg time, string frameID)
        {
#if !ROS2
            return new HeaderMsg((uint)0, time, frameID);
#else
            return new HeaderMsg(time, frameID);
#endif
        }

        TimeMsg MakeTimeMsg(int secs, int nsecs)
        {
#if !ROS2
            return new TimeMsg((uint)secs, (uint)nsecs);
#else
            return new TimeMsg(secs, (uint)nsecs);
#endif
        }

        [Test]
        public void ReceiveTF_SingleMessage_ReturnsSameTranslation()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
            TFSystem system = TFSystem.GetOrCreateInstance();
            TFSystem.TFTopicState topic = system.GetOrCreateTFTopic();
            TFStream stream = system.GetOrCreateFrame(simple_frame_id);
            TimeMsg time = MakeTimeMsg(4567, 890);
            Vector3 unityPosition = new Vector3(1, 2, 3);
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time, parent_frame_id ),
                simple_frame_id,
                new TransformMsg( unityPosition.To<FLU>(), new QuaternionMsg()
            ))}));

            TFFrame frame = stream.GetWorldTF(time.ToLongTime());
            Assert.AreEqual(frame.translation.x, unityPosition.x);
            Assert.AreEqual(frame.translation.y, unityPosition.y);
            Assert.AreEqual(frame.translation.z, unityPosition.z);
        }

        [Test]
        public void ReceiveTF_FrameWithParent_ReturnsSameTranslation()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
            TFSystem system = TFSystem.GetOrCreateInstance();
            TFSystem.TFTopicState topic = system.GetOrCreateTFTopic();
            TFStream stream = system.GetOrCreateFrame(composite_frame_id);
            Assert.AreEqual(stream.Name, simple_frame_id);
            Assert.AreEqual(stream.Parent.Name, parent_frame_id);
            TimeMsg time = MakeTimeMsg(4567, 890);
            Vector3 unityPosition = new Vector3(1, 2, 3);
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time, parent_frame_id ),
                composite_frame_id,
                new TransformMsg( unityPosition.To<FLU>(), new QuaternionMsg()
            ))}));

            TFFrame frame = stream.GetWorldTF(time.ToLongTime());
            Assert.AreEqual(frame.translation.x, unityPosition.x);
            Assert.AreEqual(frame.translation.y, unityPosition.y);
            Assert.AreEqual(frame.translation.z, unityPosition.z);
            Vector3 gameObjectPos = stream.GameObject.transform.position;
            Assert.AreEqual(gameObjectPos.x, unityPosition.x);
            Assert.AreEqual(gameObjectPos.y, unityPosition.y);
            Assert.AreEqual(gameObjectPos.z, unityPosition.z);
        }

        [Test]
        public void GetOrCreateFrame_Hierarchy_BuildsValidHierarchy()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
            TFSystem.TFTopicState topic = TFSystem.GetOrCreateInstance().GetOrCreateTFTopic();
            TFStream stream = topic.GetOrCreateFrame(composite_frame_id);
            GameObject gameObject = stream.GameObject;
            Assert.IsNotNull(stream.GameObject);
            Assert.AreEqual(stream.GameObject.name, simple_frame_id);
            Assert.AreEqual(stream.GameObject.transform.parent.name, parent_frame_id);
            TFStream stream2 = topic.GetOrCreateFrame(composite_frame_id2);
            Assert.IsNotNull(stream2.GameObject);
            Assert.AreEqual(stream2.GameObject.name, simple_frame_id2);
            Assert.AreNotEqual(stream.GameObject, stream2.GameObject);
            Assert.AreEqual(stream.GameObject.transform.parent, stream2.GameObject.transform.parent);
        }

        [Test]
        public void ReceiveTF_MultipleMessages_InterpolatesTimes()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
            TFSystem.TFTopicState topic = TFSystem.GetOrCreateInstance().GetOrCreateTFTopic();
            TFStream stream = topic.GetOrCreateFrame(simple_frame_id);
            int time1_secs = 1000;
            int time2_secs = 2000;
            TimeMsg time1 = MakeTimeMsg(time1_secs, 0);
            Vector3 unityPosition1 = new Vector3(1, 2, 3);
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time1, parent_frame_id ),
                simple_frame_id,
                new TransformMsg( unityPosition1.To<FLU>(), new QuaternionMsg()
            ))}));
            TimeMsg time2 = MakeTimeMsg(time2_secs, 0);
            Vector3 unityPosition2 = new Vector3(2, 3, 4);
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time2, parent_frame_id ),
                simple_frame_id,
                new TransformMsg( unityPosition2.To<FLU>(), new QuaternionMsg()
            ))}));
            TimeMsg time1_5 = MakeTimeMsg((time1_secs + time2_secs) / 2, 0);
            Vector3 pointAtTime1 = stream.GetWorldTF(time1).translation;
            Vector3 pointAtTime1_5 = stream.GetWorldTF(time1_5).translation;
            Vector3 pointAtTime2 = stream.GetWorldTF(time2).translation;
            Vector3 unityPosition1_5 = (unityPosition1 + unityPosition2) / 2;
            Assert.AreEqual(pointAtTime1, unityPosition1);
            Assert.AreEqual(pointAtTime1_5, unityPosition1_5);
            Assert.AreEqual(pointAtTime2, unityPosition2);
        }

        [Test]
        public void ReceiveTF_MultipleMessagesOutOfOrder_InterpolatesTimes()
        {
            ROSConnection ros = ROSConnection.GetOrCreateInstance();
            ros.ConnectOnStart = false;
            TFSystem.TFTopicState topic = TFSystem.GetOrCreateInstance().GetOrCreateTFTopic();
            TFStream stream = topic.GetOrCreateFrame(simple_frame_id);
            int time1_secs = 1000;
            int time2_secs = 2000;
            int time3_secs = 3000;
            TimeMsg time1 = MakeTimeMsg(time1_secs, 0);
            TimeMsg time2 = MakeTimeMsg(time2_secs, 0);
            TimeMsg time3 = MakeTimeMsg(time3_secs, 0);
            Vector3 unityPosition1 = new Vector3(1, 1, 1);
            Vector3 unityPosition2 = new Vector3(3, 1, 1);
            Vector3 unityPosition3 = new Vector3(3, 3, 1);
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time1, parent_frame_id ),
                simple_frame_id,
                new TransformMsg( unityPosition1.To<FLU>(), new QuaternionMsg()
            ))}));
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time3, parent_frame_id ),
                simple_frame_id,
                new TransformMsg( unityPosition3.To<FLU>(), new QuaternionMsg()
            ))}));
            topic.ReceiveTF(new TFMessageMsg(new TransformStampedMsg[] { new TransformStampedMsg(
                MakeHeaderMsg( time2, parent_frame_id ),
                simple_frame_id,
                new TransformMsg( unityPosition2.To<FLU>(), new QuaternionMsg()
            ))}));
            TimeMsg time1_5 = MakeTimeMsg((time1_secs + time2_secs) / 2, 0);
            TimeMsg time2_5 = MakeTimeMsg((time2_secs + time3_secs) / 2, 0);
            Vector3 unityPosition1_5 = (unityPosition1 + unityPosition2) / 2;
            Vector3 unityPosition2_5 = (unityPosition2 + unityPosition3) / 2;
            Assert.AreEqual(stream.GetWorldTF(time1).translation, unityPosition1);
            Assert.AreEqual(stream.GetWorldTF(time1_5).translation, unityPosition1_5);
            Assert.AreEqual(stream.GetWorldTF(time2).translation, unityPosition2);
            Assert.AreEqual(stream.GetWorldTF(time2_5).translation, unityPosition2_5);
            Assert.AreEqual(stream.GetWorldTF(time3).translation, unityPosition3);
        }
    }
}
