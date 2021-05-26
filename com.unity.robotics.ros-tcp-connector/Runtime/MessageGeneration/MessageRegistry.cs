using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public static class MessageRegistry
    {
        static Dictionary<string, Func<MessageDeserializer, Message>> s_Registry = new Dictionary<string, Func<MessageDeserializer, Message>>();
        class RegistryEntry<T>
        {
            public static Func<MessageDeserializer, T> s_Constructor;
        }

        public static void Register<T>(string rosMessageName, Func<MessageDeserializer, T> constructor) where T:Message
        {
            s_Registry[rosMessageName] = constructor;
            RegistryEntry<T>.s_Constructor = constructor;
        }

        public static Func<MessageDeserializer, Message> GetConstructor(string rosMessageName)
        {
            Func<MessageDeserializer, Message> result;
            s_Registry.TryGetValue(rosMessageName, out result);
            return result;
        }

        public static Func<MessageDeserializer, Message> GetConstructor<T>() where T:Message
        {
            return RegistryEntry<T>.s_Constructor;
        }

        public static T Deserialize<T>(MessageDeserializer deserializer) where T : Message
        {
            try
            {
                return RegistryEntry<T>.s_Constructor(deserializer);
            }
            catch(NullReferenceException)
            {
                Debug.LogError($"Message class <{typeof(T)}> exists but was not registered, it probably needs to be rebuilt.");
                return null;
            }
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void RegisterPregeneratedMessages()
        {
            MessageRegistry.Register(RosMessageTypes.BuiltinInterfaces.TimeMsg.k_RosMessageName, RosMessageTypes.BuiltinInterfaces.TimeMsg.Deserialize);
            //MessageRegistry.Register(RosMessageTypes.BuiltinInterfaces.DurationMsg.k_RosMessageName, RosMessageTypes.BuiltinInterfaces.DurationMsg.Deserialize);
            MessageRegistry.Register(RosMessageTypes.Std.HeaderMsg.k_RosMessageName, RosMessageTypes.Std.HeaderMsg.Deserialize);
        }
    }
}
