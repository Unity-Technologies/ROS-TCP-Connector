using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class RosBinaryDeserializer : IMessageDeserializer
    {
        byte[] data;
        int offset;
        public bool IsRos2 { get; private set; }
        int alignmentCorrection;

        public RosBinaryDeserializer(bool isRos2)
        {
            this.IsRos2 = isRos2;
        }

        public Message DeserializeMessage(string rosMessageName, byte[] data, MessageSubtopic subtopic = MessageSubtopic.Default)
        {
            InitWithBuffer(data);
            return MessageRegistry.GetDeserializeFunction(rosMessageName, subtopic)(this);
        }

        public T DeserializeMessage<T>(byte[] data) where T : Message
        {
            InitWithBuffer(data);
            return (T)MessageRegistry.GetDeserializeFunction<T>()(this);
        }

        public void DeserializeMessage<T>(byte[] data, out T result) where T : Message
        {
            InitWithBuffer(data);
            result = (T)MessageRegistry.GetDeserializeFunction<T>()(this);
        }

        public void InitWithBuffer(byte[] data)
        {
            this.data = data;
            this.offset = 0;
            // skip ROS2's 4 byte header
            offset = IsRos2 ? 4 : 0;
            alignmentCorrection = -offset;
        }

        void Align(int dataSize)
        {
            if (IsRos2)
                offset += (dataSize - ((offset + alignmentCorrection) % dataSize)) & (dataSize - 1);
        }

        // used for both string length, and array length
        public int ReadLength()
        {
            Align(sizeof(int));
            int result = BitConverter.ToInt32(data, offset);
            offset += sizeof(int);
            return result;
        }

        public void ReadArray<T>(out T[] values, int elementSize, int length)
        {
            if (length == 0)
            {
                values = new T[0];
                return;
            }

            Align(elementSize);
            T[] result = new T[length];
            Buffer.BlockCopy(data, offset, result, 0, length * elementSize);
            offset += elementSize * length;
            values = result;
        }

        public void Read(out bool value)
        {
            value = BitConverter.ToBoolean(data, offset);
            offset += sizeof(bool);
        }

        public void Read(out bool[] values)
        {
            ReadArray<bool>(out values, sizeof(bool), ReadLength());
        }

        public void Read(out bool[] values, int fixedLength)
        {
            ReadArray<bool>(out values, sizeof(bool), fixedLength);
        }

        public void Read(out byte value)
        {
            value = data[offset];
            offset += sizeof(byte);
        }

        public void Read(out byte[] values)
        {
            ReadArray<byte>(out values, sizeof(byte), ReadLength());
        }

        public void Read(out byte[] values, int fixedLength)
        {
            ReadArray<byte>(out values, sizeof(byte), fixedLength);
        }

        public void Read(out sbyte value)
        {
            value = (sbyte)data[offset];
            offset += sizeof(sbyte);
        }

        public void Read(out sbyte[] values)
        {
            ReadArray<sbyte>(out values, sizeof(sbyte), ReadLength());
        }

        public void Read(out sbyte[] values, int fixedLength)
        {
            ReadArray<sbyte>(out values, sizeof(sbyte), fixedLength);
        }

        public void Read(out short value)
        {
            Align(sizeof(short));
            value = BitConverter.ToInt16(data, offset);
            offset += sizeof(short);
        }

        public void Read(out short[] values)
        {
            ReadArray<short>(out values, sizeof(short), ReadLength());
        }

        public void Read(out short[] values, int fixedLength)
        {
            ReadArray<short>(out values, sizeof(short), fixedLength);
        }

        public void Read(out ushort value)
        {
            Align(sizeof(ushort));
            value = BitConverter.ToUInt16(data, offset);
            offset += sizeof(ushort);
        }

        public void Read(out ushort[] values)
        {
            ReadArray<ushort>(out values, sizeof(ushort), ReadLength());
        }

        public void Read(out ushort[] values, int fixedLength)
        {
            ReadArray<ushort>(out values, sizeof(ushort), fixedLength);
        }

        public void Read(out uint value)
        {
            Align(sizeof(uint));
            value = BitConverter.ToUInt32(data, offset);
            offset += sizeof(uint);
        }

        public void Read(out uint[] values)
        {
            ReadArray<uint>(out values, sizeof(uint), ReadLength());
        }

        public void Read(out uint[] values, int fixedLength)
        {
            ReadArray<uint>(out values, sizeof(uint), fixedLength);
        }

        public void Read(out int value)
        {
            Align(sizeof(int));
            value = BitConverter.ToInt32(data, offset);
            offset += sizeof(int);
        }

        public void Read(out int[] values)
        {
            ReadArray<int>(out values, sizeof(int), ReadLength());
        }

        public void Read(out int[] values, int fixedLength)
        {
            ReadArray<int>(out values, sizeof(int), fixedLength);
        }

        public void Read(out long value)
        {
            Align(sizeof(long));
            value = BitConverter.ToInt64(data, offset);
            offset += sizeof(long);
        }

        public void Read(out long[] values)
        {
            ReadArray<long>(out values, sizeof(long), ReadLength());
        }

        public void Read(out long[] values, int fixedLength)
        {
            ReadArray<long>(out values, sizeof(long), fixedLength);
        }

        public void Read(out ulong value)
        {
            Align(sizeof(ulong));
            value = BitConverter.ToUInt64(data, offset);
            offset += sizeof(ulong);
        }

        public void Read(out ulong[] values)
        {
            ReadArray<ulong>(out values, sizeof(ulong), ReadLength());
        }

        public void Read(out ulong[] values, int fixedLength)
        {
            ReadArray<ulong>(out values, sizeof(ulong), fixedLength);
        }

        public void Read(out float value)
        {
            Align(sizeof(float));
            value = BitConverter.ToSingle(data, offset);
            offset += sizeof(float);
        }

        public void Read(out float[] values)
        {
            ReadArray<float>(out values, sizeof(float), ReadLength());
        }

        public void Read(out float[] values, int fixedLength)
        {
            ReadArray<float>(out values, sizeof(float), fixedLength);
        }

        public void Read(out double value)
        {
            Align(sizeof(double));
            value = BitConverter.ToDouble(data, offset);
            offset += sizeof(double);
        }

        public void Read(out double[] values)
        {
            ReadArray<double>(out values, sizeof(double), ReadLength());
        }

        public void Read(out double[] values, int fixedLength)
        {
            ReadArray<double>(out values, sizeof(double), fixedLength);
        }

        public void Read(out string value)
        {
            var length = ReadLength();

            // ROS2 strings have a null byte at the end
            value = System.Text.Encoding.UTF8.GetString(data, offset, length - (IsRos2 ? 1 : 0));

            offset += length;
        }

        public void Read(out string[] values)
        {
            Read(out values, ReadLength());
        }

        public void Read(out string[] values, int length)
        {
            values = new string[length];
            for (var i = 0; i < length; i++)
            {
                Read(out values[i]);
            }
        }

        public void Read<T>(out T value) where T : Message
        {
            value = (T)MessageRegistry.GetDeserializeFunction<T>()(this);
        }

        public void Read<T>(out T[] values) where T : Message
        {
            Read<T>(out values, ReadLength());
        }

        public void Read<T>(out T[] values, int length) where T : Message
        {
            Func<IMessageDeserializer, Message> loader = MessageRegistry.GetDeserializeFunction<T>();
            values = new T[length];
            for (var i = 0; i < length; i++)
            {
                values[i] = (T)loader(this);
            }
        }

        public void BeginMessage(string[] fieldNames) { }
        public void EndMessage() { }
    }
}
