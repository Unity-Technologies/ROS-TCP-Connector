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

        static Dictionary<string, Func<IMessageDeserializer, byte[], Message>>[] s_GenericDeserializeFunctionsByName = new Dictionary<string, Func<IMessageDeserializer, byte[], Message>>[]{
            new Dictionary<string, Func<IMessageDeserializer, byte[], Message>>(), // default
            new Dictionary<string, Func<IMessageDeserializer, byte[], Message>>(), // response
            new Dictionary<string, Func<IMessageDeserializer, byte[], Message>>(), // goal
            new Dictionary<string, Func<IMessageDeserializer, byte[], Message>>(), // feedback
            new Dictionary<string, Func<IMessageDeserializer, byte[], Message>>(), // result
        };
        class RegistryEntry<T>
        {
            public static string s_RosMessageName;
            public static Func<MessageDeserializer, T> s_RosDeserializeFunction;
            public static MessageSubtopic s_Subtopic;
            public static Func<IMessageDeserializer, byte[], T> s_GenericDeserializeFunction;
        }

        public static void Register<T>(string rosMessageName, Func<MessageDeserializer, T> rosDeserialize, MessageSubtopic subtopic = MessageSubtopic.Default) where T : Message
        {
            RegistryEntry<T>.s_RosMessageName = rosMessageName;
            RegistryEntry<T>.s_RosDeserializeFunction = rosDeserialize;
            RegistryEntry<T>.s_Subtopic = subtopic;
            Func<IMessageDeserializer, byte[], T> genericDeserialize = (IMessageDeserializer deserializer, byte[] data) => deserializer.DeserializeMessage<T>(data);
            RegistryEntry<T>.s_GenericDeserializeFunction = genericDeserialize;

            if (s_RosDeserializeFunctionsByName[(int)subtopic].ContainsKey(rosMessageName))
                Debug.LogWarning($"More than one message was registered as \"{rosMessageName}\" \"{subtopic}\"");
            s_RosDeserializeFunctionsByName[(int)subtopic][rosMessageName] = rosDeserialize;
            s_GenericDeserializeFunctionsByName[(int)subtopic][rosMessageName] = genericDeserialize;
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

        public static Func<IMessageDeserializer, byte[], Message> GetGenericDeserializeFunction<T>() where T : Message
        {
            return RegistryEntry<T>.s_GenericDeserializeFunction;
        }

        public static Func<IMessageDeserializer, byte[], Message> GetGenericDeserializeFunction(string messageName, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            Func<IMessageDeserializer, byte[], Message> result;
            s_GenericDeserializeFunctionsByName[(int)subtopic].TryGetValue(messageName, out result);
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
