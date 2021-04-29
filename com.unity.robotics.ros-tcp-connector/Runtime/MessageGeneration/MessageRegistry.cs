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

        public static bool Register<T>(string typeName) where T:Message, new()
        {
            s_Constructors[typeName] = () => new T();
            return true;
        }
    }
}