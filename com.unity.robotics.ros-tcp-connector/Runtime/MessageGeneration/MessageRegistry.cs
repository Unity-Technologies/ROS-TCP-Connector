using Newtonsoft.Json.Linq;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public static class MessageRegistry
    {
        static Dictionary<string, Func<MessageDeserializer, Message>>[] s_RosDeserializeFunctionsByName = new Dictionary<string, Func<MessageDeserializer, Message>>[]{
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // default
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // response
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // goal
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // feedback
            new Dictionary<string, Func<MessageDeserializer, Message>>(), // result
        };

        static Dictionary<string, Func<JObject, Message>>[] s_JsonDeserializeFunctionsByName = new Dictionary<string, Func<JObject, Message>>[]{
            new Dictionary<string, Func<JObject, Message>>(), // default
            new Dictionary<string, Func<JObject, Message>>(), // response
            new Dictionary<string, Func<JObject, Message>>(), // goal
            new Dictionary<string, Func<JObject, Message>>(), // feedback
            new Dictionary<string, Func<JObject, Message>>(), // result
        };
        class RegistryEntry<T>
        {
            public static string s_RosMessageName;
            public static Func<MessageDeserializer, T> s_RosDeserializeFunction;
            public static MessageSubtopic s_Subtopic;
            public static Func<JObject, T> s_JsonDeserializeFunction;
        }

        public static void Register<T>(string rosMessageName, Func<MessageDeserializer, T> rosDeserialize, MessageSubtopic subtopic = MessageSubtopic.Default) where T : Message
        {
            RegistryEntry<T>.s_RosMessageName = rosMessageName;
            RegistryEntry<T>.s_RosDeserializeFunction = rosDeserialize;
            RegistryEntry<T>.s_Subtopic = subtopic;
            Func<JObject, T> genericDeserialize = (JObject json) => json.ToObject<T>();
            RegistryEntry<T>.s_JsonDeserializeFunction = genericDeserialize;

            if (s_RosDeserializeFunctionsByName[(int)subtopic].ContainsKey(rosMessageName))
                Debug.LogWarning($"More than one message was registered as \"{rosMessageName}\" \"{subtopic}\"");
            s_RosDeserializeFunctionsByName[(int)subtopic][rosMessageName] = rosDeserialize;
            s_JsonDeserializeFunctionsByName[(int)subtopic][rosMessageName] = genericDeserialize;
        }

        public static Func<MessageDeserializer, Message> GetRosDeserializeFunction(string rosMessageName, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            Func<MessageDeserializer, Message> result;
            s_RosDeserializeFunctionsByName[(int)subtopic].TryGetValue(rosMessageName, out result);
            return result;
        }

        public static Func<MessageDeserializer, Message> GetRosDeserializeFunction<T>() where T : Message
        {
            return RegistryEntry<T>.s_RosDeserializeFunction;
        }

        public static Func<JObject, Message> GetJsonDeserializeFunction<T>() where T : Message
        {
            return RegistryEntry<T>.s_JsonDeserializeFunction;
        }

        public static Func<JObject, Message> GetJsonDeserializeFunction(string messageName, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            Func<JObject, Message> result;
            s_JsonDeserializeFunctionsByName[(int)subtopic].TryGetValue(messageName, out result);
            return result;
        }

        public static string GetRosMessageName<T>() where T : Message
        {
            if (string.IsNullOrEmpty(RegistryEntry<T>.s_RosMessageName))
            {
                Debug.LogError($"Can't find MessageRegistry entry for {typeof(T)}! If Register<T> is called before " +
                               $"Start(), please specify ROS message name, or move initialization to Start().");
            }
            return RegistryEntry<T>.s_RosMessageName;
        }

        public static MessageSubtopic GetSubtopic<T>() where T : Message
        {
            return RegistryEntry<T>.s_Subtopic;
        }
    }
}
