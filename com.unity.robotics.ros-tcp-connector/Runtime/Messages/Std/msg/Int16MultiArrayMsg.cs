//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Std
{
    [Serializable]
    public class Int16MultiArrayMsg : Message
    {
        public const string k_RosMessageName = "std_msgs/Int16MultiArray";

        //  This was originally provided as an example message.
        //  It is deprecated as of Foxy
        //  It is recommended to create your own semantically meaningful message.
        //  However if you would like to continue using this please use the equivalent in example_msgs.
        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayoutMsg layout;
        //  specification of data layout
        public short[] data;
        //  array of data

        public Int16MultiArrayMsg()
        {
            this.layout = new MultiArrayLayoutMsg();
            this.data = new short[0];
        }

        public Int16MultiArrayMsg(MultiArrayLayoutMsg layout, short[] data)
        {
            this.layout = layout;
            this.data = data;
        }

        public static Int16MultiArrayMsg Deserialize(MessageDeserializer deserializer) => new Int16MultiArrayMsg(deserializer);

        private Int16MultiArrayMsg(MessageDeserializer deserializer)
        {
            this.layout = MultiArrayLayoutMsg.Deserialize(deserializer);
            deserializer.Read(out this.data, sizeof(short), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.layout);
            serializer.WriteLength(this.data);
            serializer.Write(this.data);
        }

        public override string ToString()
        {
            return "Int16MultiArrayMsg: " +
            "\nlayout: " + layout.ToString() +
            "\ndata: " + System.String.Join(", ", data.ToList());
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