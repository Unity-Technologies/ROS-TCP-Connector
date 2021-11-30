using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.MessageGeneration
{
    public class MessageDeserializer
    {
        byte[] data;
        int offset;
#if ROS2
        int alignmentCorrection;
#endif

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
#if ROS2
            // skip ROS2's 4 byte header
            offset = 4;
            alignmentCorrection = -4;
#endif
        }

        void Align(int dataSize)
        {
#if ROS2
            offset += (dataSize - ((offset + alignmentCorrection) % dataSize)) & (dataSize - 1);
#endif
        }

        public int ReadLength()
        {
            Align(sizeof(int));
            int result = BitConverter.ToInt32(data, offset);
            offset += sizeof(int);
            return result;
        }

        public void Read(out bool value)
        {
            value = BitConverter.ToBoolean(data, offset);
            offset += sizeof(bool);
        }

        public void Read(out byte value)
        {
            value = data[offset];
            offset += sizeof(byte);
        }

        public void Read(out sbyte value)
        {
            value = (sbyte)data[offset];
            offset += sizeof(sbyte);
        }

        public void Read(out short value)
        {
            Align(sizeof(short));
            value = BitConverter.ToInt16(data, offset);
            offset += sizeof(short);
        }

        public void Read(out ushort value)
        {
            Align(sizeof(ushort));
            value = BitConverter.ToUInt16(data, offset);
            offset += sizeof(ushort);
        }

        public void Read(out float value)
        {
            Align(sizeof(float));
            value = BitConverter.ToSingle(data, offset);
            offset += sizeof(float);
        }

        public void Read(out double value)
        {
            Align(sizeof(double));
            value = BitConverter.ToDouble(data, offset);
            offset += sizeof(double);
        }

        public void Read(out uint value)
        {
            Align(sizeof(uint));
            value = BitConverter.ToUInt32(data, offset);
            offset += sizeof(uint);
        }

        public void Read(out int value)
        {
            Align(sizeof(int));
            value = BitConverter.ToInt32(data, offset);
            offset += sizeof(int);
        }

        public void Read(out long value)
        {
            Align(sizeof(long));
            value = BitConverter.ToInt64(data, offset);
            offset += sizeof(long);
        }

        public void Read(out ulong value)
        {
            Align(sizeof(ulong));
            value = BitConverter.ToUInt64(data, offset);
            offset += sizeof(ulong);
        }

        public void Read(out string value)
        {
            var length = ReadLength();
#if !ROS2
            value = System.Text.Encoding.UTF8.GetString(data, offset, length);
#else
            // ROS2 strings have a null byte at the end
            value = System.Text.Encoding.UTF8.GetString(data, offset, length - 1);
#endif
            offset += length;
        }

        public void Read<T>(out T[] values, int elementSize, int length)
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

        public void Read<T>(out T[] values, Func<MessageDeserializer, Message> loader, int length) where T : Message
        {
            values = new T[length];
            for (var i = 0; i < length; i++)
            {
                values[i] = (T)loader(this);
            }
        }

        public void Read(out string[] values, int length)
        {
            values = new string[length];
            for (var i = 0; i < length; i++)
            {
                Read(out values[i]);
            }
        }
    }
}
