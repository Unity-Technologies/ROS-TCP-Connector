using Newtonsoft.Json;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector
{
    public class JsonSerializer : IMessageSerializer
    {
        public JsonSerializer(bool isRos2)
        {
            this.IsRos2 = isRos2;
            m_MessageStack = new Stack<MessageInProgress>();
            m_Builder = new StringBuilder();
        }

        public bool IsRos2 { get; private set; }
#if ROS2
        bool canUseJsonUtility => IsRos2;
#else
        bool canUseJsonUtility => !IsRos2;
#endif

        struct MessageInProgress
        {
            public string[] fieldNames;
            public int nextIndex;
        }
        Stack<MessageInProgress> m_MessageStack;
        StringBuilder m_Builder;

        void Clear()
        {
            m_MessageStack.Clear();
            m_Builder.Clear();
        }

        public string ToJsonString(Message msg)
        {
            if (canUseJsonUtility)
            {
                return JsonUtility.ToJson(msg);
            }
            else
            {
                Clear();
                msg.SerializeTo(this);
                return m_Builder.ToString();
            }
        }

        public void BeginMessage(string[] fieldNames)
        {
            m_MessageStack.Push(new MessageInProgress { fieldNames = fieldNames });
            m_Builder.Append("{");
        }

        void WriteFieldName()
        {
            MessageInProgress current = m_MessageStack.Pop();
            m_Builder.Append((current.nextIndex == 0) ? "\"" : ",\"");
            // don't need to escape the string: valid field names can't contain any weird characters
            m_Builder.Append(current.fieldNames[current.nextIndex]);
            m_Builder.Append("\":");
            current.nextIndex++;
            m_MessageStack.Push(current);
        }

        public void Write(Message msg)
        {
            WriteFieldName();
            msg.SerializeTo(this);
        }

        public void Write<T>(T[] msgs) where T : Message
        {
            WriteFieldName();
            if (msgs.Length == 0)
            {
                m_Builder.Append("[]");
            }
            else
            {
                m_Builder.Append("[");
                msgs[0].SerializeTo(this);
                for (int Idx = 1; Idx < msgs.Length; ++Idx)
                {
                    m_Builder.Append(",");
                    msgs[Idx].SerializeTo(this);
                }
                m_Builder.Append("]");
            }
        }

        public void Write<T>(T[] msgs, int fixedLength) where T : Message
        {
            Debug.Assert(msgs.Length == fixedLength);
            Write(msgs);
        }

        void WritePodArray<T>(T[] data)
        {
            WriteFieldName();
            if (data.Length == 0)
            {
                m_Builder.Append("[]");
            }
            else
            {
                m_Builder.Append("[");
                m_Builder.Append(data[0].ToString());
                for (int Idx = 1; Idx < data.Length; ++Idx)
                {
                    m_Builder.Append(",");
                    m_Builder.Append(data[Idx].ToString());
                }
                m_Builder.Append("]");
            }
        }

        public void Write(bool data)
        {
            WriteFieldName();
            m_Builder.Append(data ? "true" : "false");
        }

        public void Write(bool[] data)
        {
            WriteFieldName();
            if (data.Length == 0)
            {
                m_Builder.Append("[]");
            }
            else
            {
                m_Builder.Append("[");
                m_Builder.Append(data[0] ? "true" : "false");
                for (int Idx = 1; Idx < data.Length; ++Idx)
                {
                    m_Builder.Append(",");
                    m_Builder.Append(data[Idx] ? "true" : "false");
                }
                m_Builder.Append("]");
            }
        }

        public void Write(bool[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            Write(data);
        }

        public void Write(byte data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(byte[] data)
        {
            WritePodArray(data);
        }

        public void Write(byte[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(sbyte data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(sbyte[] data)
        {
            WritePodArray(data);
        }

        public void Write(sbyte[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(short data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }
        public void Write(short[] data)
        {
            WritePodArray(data);
        }

        public void Write(short[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(ushort data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(ushort[] data)
        {
            WritePodArray(data);
        }

        public void Write(ushort[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(int data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(int[] data)
        {
            WritePodArray(data);
        }

        public void Write(int[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(uint data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(uint[] data)
        {
            WritePodArray(data);
        }

        public void Write(uint[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(long data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(long[] data)
        {
            WritePodArray(data);
        }

        public void Write(long[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(ulong data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(ulong[] data)
        {
            WritePodArray(data);
        }

        public void Write(ulong[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(float data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(float[] data)
        {
            WritePodArray(data);
        }

        public void Write(float[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(double data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(double[] data)
        {
            WritePodArray(data);
        }

        public void Write(double[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            WritePodArray(data);
        }

        public void Write(string data)
        {
            WriteFieldName();
            m_Builder.Append(JsonConvert.ToString(data));
        }

        public void Write(string[] data)
        {
            WriteFieldName();
            if (data.Length == 0)
            {
                m_Builder.Append("[]");
            }
            else
            {
                m_Builder.Append("[");
                m_Builder.Append(JsonConvert.ToString(data));
                for (int Idx = 1; Idx < data.Length; ++Idx)
                {
                    m_Builder.Append(",");
                    m_Builder.Append(JsonConvert.ToString(data));
                }
                m_Builder.Append("]");
            }
        }

        public void Write(string[] data, int fixedLength)
        {
            Debug.Assert(data.Length == fixedLength);
            Write(data);
        }

        public void EndMessage()
        {
            m_Builder.Append("}");
            MessageInProgress oldMessage = m_MessageStack.Pop();
            Debug.Assert(oldMessage.nextIndex == oldMessage.fieldNames.Length);
        }
    }
}
