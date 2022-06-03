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
        void Write(bool data);
        void Write(bool[] data);
        void Write(bool[] data, int fixedLength);
        void Write(byte data);
        void Write(byte[] data);
        void Write(byte[] data, int fixedLength);
        void Write(sbyte data);
        void Write(sbyte[] data);
        void Write(sbyte[] data, int fixedLength);
        void Write(short data);
        void Write(short[] data);
        void Write(short[] data, int fixedLength);
        void Write(ushort data);
        void Write(ushort[] data);
        void Write(ushort[] data, int fixedLength);
        void Write(int data);
        void Write(int[] data);
        void Write(int[] data, int fixedLength);
        void Write(uint data);
        void Write(uint[] data);
        void Write(uint[] data, int fixedLength);
        void Write(long data);
        void Write(long[] data);
        void Write(long[] data, int fixedLength);
        void Write(ulong data);
        void Write(ulong[] data);
        void Write(ulong[] data, int fixedLength);
        void Write(float data);
        void Write(float[] data);
        void Write(float[] data, int fixedLength);
        void Write(double data);
        void Write(double[] data);
        void Write(double[] data, int fixedLength);
        void Write(string data);
        void Write(string[] data);
        void Write(string[] data, int fixedLength);
        void Write(Message msg);
        void Write<T>(T[] data) where T : Message;
        void Write<T>(T[] data, int fixedLength) where T : Message;
        void EndMessage();
    }
}
