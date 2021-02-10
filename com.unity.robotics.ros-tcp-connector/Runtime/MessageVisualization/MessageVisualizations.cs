using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;
using System.Reflection;

namespace Unity.Robotics.MessageVisualizers
{
    using MessageVisualizerCreator = Func<Message, MessageMetadata, IMessageVisualizerBase>;

    public interface IMessageVisualizerBase
    {
        void OnGUI();
        void End();
    }

    public interface IMessageVisualizer<MessageType> : IMessageVisualizerBase where MessageType : Message
    {
        void Begin(MessageType msg, MessageMetadata meta);
    }

    public interface IMessageVisualizer<MessageType, UserData> : IMessageVisualizerBase where MessageType : Message
    {
        void Begin(MessageType msg, MessageMetadata meta, UserData userData);
    }

    public struct MessageMetadata
    {
        public readonly string topic;
        public readonly DateTime timestamp;

        public MessageMetadata(string topic, DateTime timestamp)
        {
            this.topic = topic;
            this.timestamp = timestamp;
        }
    }

    public static class MessageVisualizations
    {
        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Point message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Point message)
        {
            GUILayout.Label($"{name}: [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Point32 message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Point32 message)
        {
            GUILayout.Label($"{name}: [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Vector3 message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Vector3 message)
        {
            GUILayout.Label($"{name}: [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Pose message, Color color, string label, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Draw<C>(drawing, message.position, color, label, size);
            UnityEngine.Vector3 point = message.position.From<C>();
            UnityEngine.Vector3 facing = message.orientation.From<C>() * UnityEngine.Vector3.forward;
            drawing.DrawLine(point, point + facing, color, size * 0.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Pose message)
        {
            GUI(name + " Position", message.position);
            GUI("Orientation", message.orientation);
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Quaternion message, UnityEngine.Vector3 position, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            UnityEngine.Quaternion quaternion = message.From<C>();
            UnityEngine.Vector3 right = quaternion * UnityEngine.Vector3.right * size;
            UnityEngine.Vector3 up = quaternion * UnityEngine.Vector3.up * size;
            UnityEngine.Vector3 forward = quaternion * UnityEngine.Vector3.forward * size;
            drawing.DrawLine(position, position + right, Color.red, size * 0.1f);
            drawing.DrawLine(position, position + up, Color.green, size * 0.1f);
            drawing.DrawLine(position, position + forward, Color.blue, size * 0.1f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Quaternion message)
        {
            GUILayout.Label($"{name}: [{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Transform transform, float size = 0.01f) where C : ICoordinateSpace, new()
        {
            Draw<C>(drawing, transform.rotation, transform.translation.From<C>(), size);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Transform message)
        {
            GUI(name + " Translation", message.translation);
            GUI("Rotation", message.rotation);
        }

        static bool initialized;
        private static Dictionary<string, MessageVisualizerCreator> TopicVisualizers = new Dictionary<string, MessageVisualizerCreator>();
        private static Dictionary<Type, MessageVisualizerCreator> TypeVisualizers = new Dictionary<Type, MessageVisualizerCreator>();

        public static void InitAllVisualizers()
        {
            if (initialized)
                return;

            initialized = true;
            MethodInfo genericCreator = typeof(PrivateFunctions).GetMethod("GetCreator");
            Dictionary<System.Type, int> messageTypePriorities = new Dictionary<Type, int>();

            // find and register all classes with the AutoRegisteredVisualizer attribute
            foreach (Assembly a in AppDomain.CurrentDomain.GetAssemblies())
            {
                foreach (Type classType in a.GetTypes())
                {
                    foreach (AutoRegisteredVisualizerAttribute attr in classType.GetCustomAttributes(typeof(AutoRegisteredVisualizerAttribute), false))
                    {
                        Type messageVisualizerGeneric = classType.GetInterface("IMessageVisualizer`1");
                        if(messageVisualizerGeneric == null)
                        {
                            Type messageVisualizerUserdata = classType.GetInterface("IMessageVisualizer`2");
                            if (messageVisualizerUserdata == null)
                                Debug.LogError($"Class {classType} cannot be an AutoRegisteredVisualizer - it doesn't implement IMessageVisualizer!");
                            else
                                Debug.LogError($"Class {classType} cannot be an AutoRegisteredVisualizer - it requires a UserData value");
                            break;
                        }

                        Type messageType = messageVisualizerGeneric.GenericTypeArguments[0];
                        MethodInfo getCreator = genericCreator.MakeGenericMethod(
                            new Type[] { classType, messageType }
                        );

                        if (attr.topic != null)
                        {
                            TopicVisualizers.Add(attr.topic, (MessageVisualizerCreator)getCreator.Invoke(null, null));
                        }
                        else
                        {
                            int priority;
                            if(!messageTypePriorities.TryGetValue(messageType, out priority) || attr.priority > priority)
                            {
                                TypeVisualizers.Add(messageType, (MessageVisualizerCreator)getCreator.Invoke(null, null));
                                messageTypePriorities[messageType] = attr.priority;
                            }
                        }
                    }
                }
            }
        }

        public static void RegisterVisualizer<VisualizerType, MsgType>(string topic)
            where VisualizerType : IMessageVisualizer<MsgType>, new()
            where MsgType : Message
        {
            InitAllVisualizers();
            TopicVisualizers.Add(topic, PrivateFunctions.GetCreator<VisualizerType, MsgType>());
        }

        public static void RegisterVisualizer<VisualizerType, MsgType>(System.Type messageType)
            where VisualizerType : IMessageVisualizer<MsgType>, new()
            where MsgType : Message
        {
            InitAllVisualizers();
            TypeVisualizers.Add(messageType, PrivateFunctions.GetCreator<VisualizerType, MsgType>());
        }

        public static void RegisterVisualizer<VisualizerType, MsgType, UserData>(string topic, UserData userData)
            where VisualizerType : IMessageVisualizer<MsgType, UserData>, new()
            where MsgType : Message
        {
            InitAllVisualizers();
            TopicVisualizers.Add(topic, PrivateFunctions.GetCreatorWithUserdata<VisualizerType, MsgType, UserData>(userData));
        }

        public static void RegisterVisualizer<VisualizerType, MsgType, UserData>(UserData userData)
            where VisualizerType : IMessageVisualizer<MsgType, UserData>, new()
            where MsgType : Message
        {
            InitAllVisualizers();
            TypeVisualizers.Add(typeof(MsgType), PrivateFunctions.GetCreatorWithUserdata<VisualizerType, MsgType, UserData>(userData));
        }

        public static IMessageVisualizerBase GetVisualizer(Message message, MessageMetadata meta)
        {
            MessageVisualizerCreator result;
            if (TopicVisualizers.TryGetValue(meta.topic, out result))
                return result(message, meta);

            if (TypeVisualizers.TryGetValue(message.GetType(), out result))
                return result(message, meta);

            DefaultVisualizer defaultVisualizer = new DefaultVisualizer();
            defaultVisualizer.Begin(message, meta);
            return defaultVisualizer;
        }

        class DefaultVisualizer : IMessageVisualizer<Message>
        {
            Message message;
            string messageString;
            public void Begin(Message message, MessageMetadata meta)
            {
                this.message = message;
            }

            public void OnGUI()
            {
                if(messageString == null)
                    messageString = message.ToString();
                GUILayout.Label(messageString);
            }

            public void End()
            {
            }
        }

        private static class PrivateFunctions
        {
            public static MessageVisualizerCreator GetCreator<VisualizerType, MsgType>()
            where VisualizerType : IMessageVisualizer<MsgType>, new()
            where MsgType : Message
            {
                return (Message msg, MessageMetadata meta) =>
                {
                    IMessageVisualizer<MsgType> result = new VisualizerType();
                    result.Begin((MsgType)msg, meta);
                    return result;
                };
            }

            public static MessageVisualizerCreator GetCreatorWithUserdata<VisualizerType, MsgType, UserData>(UserData userData)
                where VisualizerType : IMessageVisualizer<MsgType, UserData>, new()
                where MsgType : Message
            {
                return (Message msg, MessageMetadata meta) =>
                {
                    IMessageVisualizer<MsgType, UserData> result = new VisualizerType();
                    result.Begin((MsgType)msg, meta, userData);
                    return result;
                };
            }
        }
    }
}