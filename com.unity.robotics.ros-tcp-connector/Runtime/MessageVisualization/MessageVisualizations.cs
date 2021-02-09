using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;
using System.Reflection;

namespace Unity.Robotics.MessageVisualizers
{
    using MessageVisualizerCreator = Func<string, Message, IMessageVisualizerBase>;

    public interface IMessageVisualizerBase
    {
        void OnGUI();
        void End();
    }

    public interface IMessageVisualizer<MessageType> : IMessageVisualizerBase where MessageType : Message
    {
        void Begin(string topic, MessageType msg);
    }

    public interface IMessageVisualizer<MessageType, UserData> : IMessageVisualizerBase where MessageType : Message
    {
        void Begin(string topic, MessageType msg, UserData userData);
    }

    public static class MessageVisualizations
    {
        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Point message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Point message)
        {
            GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Point32 message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Point32 message)
        {
            GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Vector3 message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            drawing.DrawPoint(message.From<C>(), color, size);
            drawing.DrawLabel(label, message.From<C>(), color, size * 1.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Vector3 message)
        {
            GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Pose message, Color color, string label, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            Draw<C>(drawing, message.position, color, label, size);
            UnityEngine.Vector3 point = message.position.From<C>();
            UnityEngine.Vector3 facing = message.orientation.From<C>() * UnityEngine.Vector3.forward;
            drawing.DrawLine(point, point + facing, color, size * 0.5f);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Pose message)
        {
            GUI(name + " - Position", message.position);
            GUI("Orientation", message.orientation);
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Quaternion message, UnityEngine.Vector3 position, float size = 0.1f) where C : ICoordinateSpace, new()
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
            GUILayout.Label($"{name} - [{message.x:F2}, {message.y:F2}, {message.z:F2}, {message.w:F2}]");
        }

        public static void Draw<C>(DebugDraw.Drawing drawing, RosMessageTypes.Geometry.Transform transform, float size = 0.1f) where C : ICoordinateSpace, new()
        {
            Draw<C>(drawing, transform.rotation, transform.translation.From<C>(), size);
        }

        public static void GUI(string name, RosMessageTypes.Geometry.Transform message)
        {
            GUI(name + " - Translation", message.translation);
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
            MethodInfo genericCreator = typeof(VisualizerCreators).GetMethod("GetCreatorTM");

            // find and register all classes with the VisualizeMessage attribute
            foreach (Assembly a in AppDomain.CurrentDomain.GetAssemblies())
            {
                foreach (Type classType in a.GetTypes())
                {
                    if (!typeof(IMessageVisualizerBase).IsAssignableFrom(classType))
                        continue;

                    foreach (RegisterVisualizerAttribute attr in classType.GetCustomAttributes(typeof(RegisterVisualizerAttribute), false))
                    {
                        Type messageVisualizerGeneric = classType.GetInterface("IMessageVisualizer`1");
                        Type messageType = messageVisualizerGeneric.GenericTypeArguments[0];
                        MethodInfo getCreator = genericCreator.MakeGenericMethod(
                            new Type[] { classType, messageType });

                        if (attr.topic != null)
                            TopicVisualizers.Add(attr.topic, (MessageVisualizerCreator)getCreator.Invoke(null, null));
                        else
                            TypeVisualizers.Add(messageType, (MessageVisualizerCreator)getCreator.Invoke(null, null));
                    }
                }
            }
        }

        public static void RegisterVisualizer<VisualizerType, MsgType>(string topic)
            where VisualizerType : IMessageVisualizer<MsgType>, new()
            where MsgType : Message
        {
            TopicVisualizers.Add(topic, VisualizerCreators.GetCreatorTM<VisualizerType, MsgType>());
        }

        public static void RegisterVisualizer<VisualizerType, MsgType>(System.Type messageType)
            where VisualizerType : IMessageVisualizer<MsgType>, new()
            where MsgType : Message
        {
            TypeVisualizers.Add(messageType, VisualizerCreators.GetCreatorTM<VisualizerType, MsgType>());
        }

        public static void RegisterVisualizer<VisualizerType, MsgType, UserData>(string topic, UserData userData)
            where VisualizerType : IMessageVisualizer<MsgType, UserData>, new()
            where MsgType : Message
        {
            TopicVisualizers.Add(topic, VisualizerCreators.GetCreatorTMU<VisualizerType, MsgType, UserData>(userData));
        }

        public static void RegisterVisualizer<VisualizerType, MsgType, UserData>(UserData userData)
            where VisualizerType : IMessageVisualizer<MsgType, UserData>, new()
            where MsgType : Message
        {
            TypeVisualizers.Add(typeof(MsgType), VisualizerCreators.GetCreatorTMU<VisualizerType, MsgType, UserData>(userData));
        }

        public static IMessageVisualizerBase GetVisualizer(string topic, Message message)
        {
            MessageVisualizerCreator result;
            if (TopicVisualizers.TryGetValue(topic, out result))
                return result(topic, message);

            if (TypeVisualizers.TryGetValue(message.GetType(), out result))
                return result(topic, message);

            DefaultVisualizer defaultVisualizer = new DefaultVisualizer();
            defaultVisualizer.Begin(topic, message);
            return defaultVisualizer;
        }

        class DefaultVisualizer : IMessageVisualizer<Message>
        {
            Message message;
            string messageString;
            public void Begin(string topic, Message message)
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

        private static class VisualizerCreators
        {
            public static MessageVisualizerCreator GetCreatorTM<VisualizerType, MsgType>()
            where VisualizerType : IMessageVisualizer<MsgType>, new()
            where MsgType : Message
            {
                return (string tpc, Message msg) =>
                {
                    IMessageVisualizer<MsgType> result = new VisualizerType();
                    result.Begin(tpc, (MsgType)msg);
                    return result;
                };
            }

            public static MessageVisualizerCreator GetCreatorTMU<VisualizerType, MsgType, UserData>(UserData userData)
                where VisualizerType : IMessageVisualizer<MsgType, UserData>, new()
                where MsgType : Message
            {
                return (string tpc, Message msg) =>
                {
                    IMessageVisualizer<MsgType, UserData> result = new VisualizerType();
                    result.Begin(tpc, (MsgType)msg, userData);
                    return result;
                };
            }
        }
    }
}