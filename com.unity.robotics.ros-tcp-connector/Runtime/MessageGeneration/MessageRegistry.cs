using Newtonsoft.Json.Linq;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public static class MessageRegistry
    {
        struct RegistryDataByName
        {
            public Func<IMessageDeserializer, Message> m_Deserialize;
            public Func<IGenericInvokable, object, Message> m_GenericInvoke;
        }

        static Dictionary<string, RegistryDataByName>[] s_RegistryByName = new Dictionary<string, RegistryDataByName>[]{
            new Dictionary<string, RegistryDataByName>(), // default
            new Dictionary<string, RegistryDataByName>(), // response
            new Dictionary<string, RegistryDataByName>(), // goal
            new Dictionary<string, RegistryDataByName>(), // feedback
            new Dictionary<string, RegistryDataByName>(), // result
        };

        class RegistryEntry<T>
        {
            public static string s_MessageType;
            public static Func<IMessageDeserializer, T> s_DeserializeFunction;
            public static MessageSubtopic s_Subtopic;
        }

        public static void Register<T>(string messageType, Func<MessageDeserializer, T> rosDeserialize, MessageSubtopic subtopic = MessageSubtopic.Default) where T : Message
        {
            // TODO: throw NotImplementedException here once generator is working
            RegistryEntry<T>.s_MessageType = messageType;
            RegistryEntry<T>.s_Subtopic = subtopic;
        }

        public static void Register<T>(string messageType, Func<IMessageDeserializer, T> deserialize, MessageSubtopic subtopic = MessageSubtopic.Default) where T : Message
        {
            RegistryEntry<T>.s_MessageType = messageType;
            RegistryEntry<T>.s_DeserializeFunction = deserialize;
            RegistryEntry<T>.s_Subtopic = subtopic;

            if (s_RegistryByName[(int)subtopic].ContainsKey(messageType))
                Debug.LogWarning($"More than one message was registered as \"{messageType}\" \"{subtopic}\"");
            s_RegistryByName[(int)subtopic][messageType] = new RegistryDataByName
            {
                m_Deserialize = deserialize,
                m_GenericInvoke = (invokable, arg) => invokable.Invoke<T>(arg)
            };
        }

        public static Func<IMessageDeserializer, Message> GetDeserializeFunction(string messageType, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            RegistryDataByName data;
            if (!s_RegistryByName[(int)subtopic].TryGetValue(messageType, out data))
            {
                Debug.LogError($"Can't find MessageRegistry entry for '{messageType}'!");
            }
            return data.m_Deserialize;
        }

        public static Func<IMessageDeserializer, Message> GetDeserializeFunction<T>() where T : Message
        {
            return RegistryEntry<T>.s_DeserializeFunction;
        }

        public static string GetMessageTypeString<T>() where T : Message
        {
            if (string.IsNullOrEmpty(RegistryEntry<T>.s_MessageType))
            {
                Debug.LogError($"Can't find MessageRegistry entry for {typeof(T)}! If Register<T> is called before " +
                               $"Start(), please specify ROS message name, or move initialization to Start().");
            }
            return RegistryEntry<T>.s_MessageType;
        }

        public static MessageSubtopic GetSubtopic<T>() where T : Message
        {
            return RegistryEntry<T>.s_Subtopic;
        }

        public interface IGenericInvokable
        {
            Message Invoke<T>(object arg) where T : Message;
        }

        public static Message InvokeGeneric(IGenericInvokable invokable, object arg, string messageType, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            Func<IGenericInvokable, object, Message> invoker = GetGenericInvoker(messageType, subtopic);
            if (invoker != null)
                return invoker(invokable, arg);
            else
                return null;
        }

        public static Func<IGenericInvokable, object, Message> GetGenericInvoker(string messageType, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            RegistryDataByName data;
            if (!s_RegistryByName[(int)subtopic].TryGetValue(messageType, out data))
            {
                Debug.LogError($"Can't find MessageRegistry entry for '{messageType}'!");
                return null;
            }
            return data.m_GenericInvoke;
        }
    }
}
