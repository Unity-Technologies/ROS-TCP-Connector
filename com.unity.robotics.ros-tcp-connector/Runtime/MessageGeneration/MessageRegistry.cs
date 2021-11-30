using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public static class MessageRegistry
    {
        static Dictionary<string, Func<MessageDeserializer, Message>>[] s_DeserializeFunctionsByName = new Dictionary<string, Func<MessageDeserializer, Message>>[]{
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // default
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // response
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // goal
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // feedback
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // result
        };
        class RegistryEntry<T>
        {
            public static string s_RosMessageName;
            public static Func<MessageDeserializer, T> s_DeserializeFunction;
            public static MessageSubtopic s_Subtopic;
        }

        public static void Register<T>(string rosMessageName, Func<MessageDeserializer, T> deserialize, MessageSubtopic subtopic = MessageSubtopic.Default) where T : Message
        {
            RegistryEntry<T>.s_RosMessageName = rosMessageName;
            RegistryEntry<T>.s_DeserializeFunction = deserialize;
            RegistryEntry<T>.s_Subtopic = subtopic;
            if (s_DeserializeFunctionsByName[(int)subtopic].ContainsKey(rosMessageName))
                Debug.LogWarning($"More than one message was registered as \"{rosMessageName}\" \"{subtopic}\"");
            s_DeserializeFunctionsByName[(int)subtopic][rosMessageName] = deserialize;
        }

        public static Func<MessageDeserializer, Message> GetDeserializeFunction(string rosMessageName, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            Func<MessageDeserializer, Message> result;
            s_DeserializeFunctionsByName[(int)subtopic].TryGetValue(rosMessageName, out result);
            return result;
        }

        public static Func<MessageDeserializer, Message> GetDeserializeFunction<T>() where T : Message
        {
            return RegistryEntry<T>.s_DeserializeFunction;
        }

        public static string GetRosMessageName<T>() where T : Message
        {
            return RegistryEntry<T>.s_RosMessageName;
        }

        public static MessageSubtopic GetSubtopic<T>() where T : Message
        {
            return RegistryEntry<T>.s_Subtopic;
        }
    }
}
