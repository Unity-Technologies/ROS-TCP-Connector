using System.IO;
using JetBrains.Annotations;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public abstract class SysCommand
    {
        public const string k_SysCommand_Log = "__log";
        public const string k_SysCommand_Warning = "__warn";
        public const string k_SysCommand_Error = "__error";
        public const string k_SysCommand_ServiceRequest = "__request";
        public const string k_SysCommand_ServiceResponse = "__response";
        public const string k_SysCommand_Subscribe = "__subscribe";
        public const string k_SysCommand_Publish = "__publish";
        public const string k_SysCommand_RosService = "__ros_service";
        public const string k_SysCommand_UnityService = "__unity_service";
        public const string k_SysCommand_TopicList = "__topic_list";
        public const string k_SysCommand_RemoveSubscriber = "__remove_subscriber";
        public const string k_SysCommand_RemovePublisher = "__remove_publisher";
        public const string k_SysCommand_RemoveRosService = "__remove_ros_service";
        public const string k_SysCommand_RemoveUnityService = "__remove_unity_service";

        public abstract string Command
        {
            get;
        }

        public virtual object BuildParam()
        {
            return this;

        }
        public void PopulateSysCommand(MessageSerializer messageSerializer)
        {
            messageSerializer.Clear();
            // syscommands are sent as:
            // 4 byte command length, followed by that many bytes of the command
            // (all command names start with __ to distinguish them from ros topics)
            messageSerializer.Write(Command);
            // 4-byte json length, followed by a json string of that length
            string json = JsonUtility.ToJson(BuildParam());
            messageSerializer.WriteUnaligned(json);
        }

        public void SendTo([NotNull] Stream stream, MessageSerializer messageSerializer = null)
        {
            if (messageSerializer == null)
            {
                messageSerializer = new MessageSerializer();
            }

            PopulateSysCommand(messageSerializer);
            messageSerializer.SendTo(stream);
        }
    }

    public struct SysCommand_Topic
    {
        public string topic;
    }

    public struct SysCommand_TopicAndType
    {
        public string topic;
        public string message_name;
    }

    public struct SysCommand_Log
    {
        public string text;
    }

    public struct SysCommand_Service
    {
        public int srv_id;
    }

    public struct SysCommand_TopicsRequest
    {
    }

    public struct SysCommand_TopicsResponse
    {
        public string[] topics;
        public string[] types;
    }

    public struct SysCommand_PublisherRegistration
    {
        public string topic;
        public string message_name;
        public int queue_size;
        public bool latch;
    }
}
