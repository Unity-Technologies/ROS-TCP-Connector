using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public interface IMessageSerializer
    {
        //byte[] SerializeMessage(string topic, Message msg);
        //void SendMessage(string topic, Message msg, System.IO.Stream outStream);
        bool IsRos2 { get; }
        void BeginMessage(string[] fieldNames);
        void Write(Message msg);
        void Write(string data);
        void Write(uint data);
        void Write(int data);
        void Write(double[] data);
        void Write(double[] data, int fixedLength);
        void Write(bool data);
        void EndMessage();
    }
}
