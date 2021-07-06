//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Geometry
{
    [Serializable]
    public class QuaternionStampedMsg : Message
    {
        public const string k_RosMessageName = "geometry_msgs/QuaternionStamped";

        //  This represents an orientation with reference coordinate frame and timestamp.
        public Std.HeaderMsg header;
        public QuaternionMsg quaternion;

        public QuaternionStampedMsg()
        {
            this.header = new Std.HeaderMsg();
            this.quaternion = new QuaternionMsg();
        }

        public QuaternionStampedMsg(Std.HeaderMsg header, QuaternionMsg quaternion)
        {
            this.header = header;
            this.quaternion = quaternion;
        }

        public static QuaternionStampedMsg Deserialize(MessageDeserializer deserializer) => new QuaternionStampedMsg(deserializer);

        private QuaternionStampedMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            this.quaternion = QuaternionMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.quaternion);
        }

        public override string ToString()
        {
            return "QuaternionStampedMsg: " +
            "\nheader: " + header.ToString() +
            "\nquaternion: " + quaternion.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
