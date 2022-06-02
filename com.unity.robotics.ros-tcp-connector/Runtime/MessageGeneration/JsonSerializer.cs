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

        public string GetJsonString(Message msg)
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
            if (m_MessageStack.Count > 0)
                WriteFieldName();
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

        [System.Serializable]
        struct StringContainer
        {
            public string s;
        }

        public void Write(string data)
        {
            WriteFieldName();
            m_Builder.Append('"' + data + '"'); // JsonConvert.ToString(data));
            //string escapedJson = JsonUtility.ToJson(new StringContainer { s = data });
            //string escaped = escapedJson.Substring(5, escapedJson.Length-6);
            //m_Builder.Append(escaped);
            //m_Builder.Append(JsonConvert.ToString(data));
        }

        public void Write(uint data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(int data)
        {
            WriteFieldName();
            m_Builder.Append(data);
        }

        public void Write(bool data)
        {
            WriteFieldName();
            m_Builder.Append(data ? "true" : "false");
        }

        public void Write(double[] data)
        {
            WriteFieldName();
            if (data.Length == 0)
            {
                m_Builder.Append("[]");
            }
            else
            {
                m_Builder.Append("[");
                m_Builder.Append(data[0]);
                for (int Idx = 1; Idx < data.Length; ++Idx)
                {
                    m_Builder.Append(",");
                    m_Builder.Append(data[Idx]);
                }
                m_Builder.Append("]");
            }
        }

        public void EndMessage()
        {
            m_Builder.Append("}");
            MessageInProgress oldMessage = m_MessageStack.Pop();
            Debug.Assert(oldMessage.nextIndex == oldMessage.fieldNames.Length);
        }
    }
}
