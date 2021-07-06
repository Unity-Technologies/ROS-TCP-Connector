//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Tf2
{
    [Serializable]
    public class TF2ErrorMsg : Message
    {
        public const string k_RosMessageName = "tf2_msgs/TF2Error";

        public const byte NO_ERROR = 0;
        public const byte LOOKUP_ERROR = 1;
        public const byte CONNECTIVITY_ERROR = 2;
        public const byte EXTRAPOLATION_ERROR = 3;
        public const byte INVALID_ARGUMENT_ERROR = 4;
        public const byte TIMEOUT_ERROR = 5;
        public const byte TRANSFORM_ERROR = 6;
        public byte error;
        public string error_string;

        public TF2ErrorMsg()
        {
            this.error = 0;
            this.error_string = "";
        }

        public TF2ErrorMsg(byte error, string error_string)
        {
            this.error = error;
            this.error_string = error_string;
        }

        public static TF2ErrorMsg Deserialize(MessageDeserializer deserializer) => new TF2ErrorMsg(deserializer);

        private TF2ErrorMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.error);
            deserializer.Read(out this.error_string);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.error);
            serializer.Write(this.error_string);
        }

        public override string ToString()
        {
            return "TF2ErrorMsg: " +
            "\nerror: " + error.ToString() +
            "\nerror_string: " + error_string.ToString();
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
