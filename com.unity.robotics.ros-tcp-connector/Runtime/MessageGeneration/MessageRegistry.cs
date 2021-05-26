using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public static class MessageRegistry
    {
        static Dictionary<string, Func<MessageDeserializer, Message>> s_ConstructorsByName = new Dictionary<string, Func<MessageDeserializer, Message>>();
        class RegistryEntry<T>
        {
            public static string s_RosMessageName;
            public static Func<MessageDeserializer, T> s_Constructor;
        }

        public static void Register<T>(string rosMessageName, Func<MessageDeserializer, T> constructor) where T : Message
        {
            RegistryEntry<T>.s_RosMessageName = rosMessageName;
            RegistryEntry<T>.s_Constructor = constructor;
            s_ConstructorsByName[rosMessageName] = constructor;
        }

        public static Func<MessageDeserializer, Message> GetConstructor(string rosMessageName)
        {
            Func<MessageDeserializer, Message> result;
            s_ConstructorsByName.TryGetValue(rosMessageName, out result);
            return result;
        }

        public static Func<MessageDeserializer, Message> GetConstructor<T>() where T : Message
        {
            return RegistryEntry<T>.s_Constructor;
        }

        public static string GetRosMessageName<T>() where T : Message
        {
            return RegistryEntry<T>.s_RosMessageName;
        }
    }
}
