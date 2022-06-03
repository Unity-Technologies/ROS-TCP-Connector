using System.Collections.Generic;
using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.ROSTCPConnector
{
    public abstract class OutgoingMessageSender
    {

        public enum SendToState
        {
            Normal,
            NoMessageToSendError,
            QueueFullWarning
        }

        public abstract SendToState SendInternal(RosBinarySerializer messageSerializer, System.IO.Stream stream);

        public abstract void ClearAllQueuedData();
    }

    /**
     * Simple implementation of a OutgoingMessageSender that is used for sys commands
     * as they are handled differently to typical ROS messages and sent as JSON strings.
     */
    public class SysCommandSender : OutgoingMessageSender
    {
        string command;
        string json;

        public SysCommandSender(string command, object param)
        {
            this.command = command;
            this.json = UnityEngine.JsonUtility.ToJson(param);
        }

        public override SendToState SendInternal(RosBinarySerializer messageSerializer, Stream stream)
        {
            if (json != null)
            {
                messageSerializer.SendString(command, json, stream);
            }
            return SendToState.Normal;
        }

        public override void ClearAllQueuedData()
        {
            json = null;
        }
    }
}
