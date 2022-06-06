using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public interface IMessageDeserializer
    {
        bool IsRos2 { get; }
        void BeginMessage(string[] fieldNames);
        void Read(out bool data);
        void Read(out bool[] data);
        void Read(out bool[] data, int fixedLength);
        void Read(out byte data);
        void Read(out byte[] data);
        void Read(out byte[] data, int fixedLength);
        void Read(out sbyte data);
        void Read(out sbyte[] data);
        void Read(out sbyte[] data, int fixedLength);
        void Read(out short data);
        void Read(out short[] data);
        void Read(out short[] data, int fixedLength);
        void Read(out ushort data);
        void Read(out ushort[] data);
        void Read(out ushort[] data, int fixedLength);
        void Read(out int data);
        void Read(out int[] data);
        void Read(out int[] data, int fixedLength);
        void Read(out uint data);
        void Read(out uint[] data);
        void Read(out uint[] data, int fixedLength);
        void Read(out long data);
        void Read(out long[] data);
        void Read(out long[] data, int fixedLength);
        void Read(out ulong data);
        void Read(out ulong[] data);
        void Read(out ulong[] data, int fixedLength);
        void Read(out float data);
        void Read(out float[] data);
        void Read(out float[] data, int fixedLength);
        void Read(out double data);
        void Read(out double[] data);
        void Read(out double[] data, int fixedLength);
        void Read(out string data);
        void Read(out string[] data);
        void Read(out string[] data, int fixedLength);
        void Read<T>(out T data) where T : Message;
        void Read<T>(out T[] data) where T : Message;
        void Read<T>(out T[] data, int fixedLength) where T : Message;
        void EndMessage();
    }
}
