using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class MessageRegistry: ScriptableObject
    {
        public string[] tableNames;

        static MessageRegistry instance;

        static void LoadInstance()
        {
            if (instance != null)
                return;
            instance = Resources.Load<MessageRegistry>("MessageRegistry");
#if UNITY_EDITOR
            if (instance == null)
            {
                instance = ScriptableObject.CreateInstance<MessageRegistry>();
                if (!Directory.Exists("Assets/Resources"))
                    Directory.CreateDirectory("Assets/Resources");
                UnityEditor.AssetDatabase.CreateAsset(instance, "Assets/Resources/MessageRegistry.asset");
            }
#endif
        }

#if UNITY_EDITOR
        public static void Build()
        {
            if (instance == null)
                LoadInstance();

            List<string> tableNames = new List<string>();
            foreach(System.Reflection.Assembly a in AppDomain.CurrentDomain.GetAssemblies())
            {
                Type table = a.GetType("Unity.Robotics.ROSTCPConnector.MessageGeneration.AssemblyMessagesTable");
                if (table == null)
                    continue;

                tableNames.Add(table.AssemblyQualifiedName);
            }
            instance.tableNames = tableNames.ToArray();
            UnityEditor.EditorUtility.SetDirty(instance);
            UnityEditor.AssetDatabase.Refresh();
        }
#endif

        public static Func<Message> GetConstructor(string rosMessageName)
        {
            if (s_Constructors == null)
            {
                if (instance == null)
                    LoadInstance();

                s_Constructors = new Dictionary<string, Func<Message>>();
                foreach (string tableName in instance.tableNames)
                {
                    Type tableType = Type.GetType(tableName);
                    System.Runtime.CompilerServices.RuntimeHelpers.RunClassConstructor(tableType.TypeHandle);
                }
            }

            Func<Message> constructorFunc;
            s_Constructors.TryGetValue(rosMessageName, out constructorFunc);

            return constructorFunc;
        }

        static Dictionary<string, Func<Message>> s_Constructors;
        public static bool Register<T>(string typeName) where T:Message, new()
        {
            Func<Message> constructor = () => new T();
            s_Constructors.Add(typeName, constructor);
            return true;
        }
    }
}