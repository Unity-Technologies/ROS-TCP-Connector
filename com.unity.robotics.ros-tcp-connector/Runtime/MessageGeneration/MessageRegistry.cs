using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class MessageRegistry
    {
        static Dictionary<string, Func<Message>> s_Constructors = new Dictionary<string, Func<Message>>();

        public static Func<Message> GetConstructor(string rosMessageName)
        {
            Func<Message> constructorFunc;
            s_Constructors.TryGetValue(rosMessageName, out constructorFunc);

            return constructorFunc;
        }

        public static string GetMessageName<T>() where T : Message
        {
            return RegistryEntry<T>.typeName;
        }

        public static bool Register<T>(string typeName) where T:Message, new()
        {
            s_Constructors[typeName] = () => new T();
            RegistryEntry<T>.typeName = typeName;
            return true;
        }

        class RegistryEntry<T>
        {
            public static string typeName = null;
        }
    }
}