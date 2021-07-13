using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public static class MessageRegistry
    {
        static Dictionary<string, Func<MessageDeserializer, Message>> s_DeserializeFunctionsByName =
            new Dictionary<string, Func<MessageDeserializer, Message>>();

        static class RegistryEntry<T>
        {
            public static string s_RosMessageName;
            public static Func<MessageDeserializer, T> s_DeserializeFunction;
        }

        public static void Register<T>(string rosMessageName, Func<MessageDeserializer, T> deserialize) where T : Message
        {
            RegistryEntry<T>.s_RosMessageName = rosMessageName;
            RegistryEntry<T>.s_DeserializeFunction = deserialize;
            s_DeserializeFunctionsByName[rosMessageName] = deserialize;
        }

        public static bool HasDeserializeFunction(string rosMessageName)
        {
            return s_DeserializeFunctionsByName.ContainsKey(rosMessageName);
        }

        public static Func<MessageDeserializer, Message> GetDeserializeFunction(string rosMessageName)
        {
            Func<MessageDeserializer, Message> result;
            s_DeserializeFunctionsByName.TryGetValue(rosMessageName, out result);
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
    }
}
