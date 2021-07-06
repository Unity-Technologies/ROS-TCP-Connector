//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.ObjectRecognition
{
    [Serializable]
    public class ObjectRecognitionResult : Message
    {
        public const string k_RosMessageName = "object_recognition_msgs-master/ObjectRecognition";

        //  Send the found objects, see the msg files for docs
        public RecognizedObjectArrayMsg recognized_objects;

        public ObjectRecognitionResult()
        {
            this.recognized_objects = new RecognizedObjectArrayMsg();
        }

        public ObjectRecognitionResult(RecognizedObjectArrayMsg recognized_objects)
        {
            this.recognized_objects = recognized_objects;
        }

        public static ObjectRecognitionResult Deserialize(MessageDeserializer deserializer) => new ObjectRecognitionResult(deserializer);

        private ObjectRecognitionResult(MessageDeserializer deserializer)
        {
            this.recognized_objects = RecognizedObjectArrayMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.recognized_objects);
        }

        public override string ToString()
        {
            return "ObjectRecognitionResult: " +
            "\nrecognized_objects: " + recognized_objects.ToString();
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
