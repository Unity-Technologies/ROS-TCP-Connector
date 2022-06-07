using Newtonsoft.Json.Linq;
using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class JsonDeserializer : IMessageDeserializer, MessageRegistry.IGenericInvokable
{
    public bool IsRos2 { get; private set; }
#if ROS2
    bool canUseJsonUtility => IsRos2;
#else
    bool canUseJsonUtility => !IsRos2;
#endif

    struct MessageInProgress
    {
        public JToken[] values;
        public int nextIndex;
        public int numValues;
        public JToken nextValue => values[nextIndex];
    }
    Stack<MessageInProgress> m_MessageStack = new Stack<MessageInProgress>();
    JObject m_Root;

    public JsonDeserializer(bool isRos2)
    {
        IsRos2 = isRos2;
    }

    public Message DeserializeMessage(string json, string messageType, MessageSubtopic subtopic = MessageSubtopic.Default)
    {
        return MessageRegistry.InvokeGeneric(this, json, messageType, subtopic);
    }

    public Message DeserializeMessage(JObject json, string messageType, MessageSubtopic subtopic = MessageSubtopic.Default)
    {
        return MessageRegistry.InvokeGeneric(this, json, messageType, subtopic);
    }

    Message MessageRegistry.IGenericInvokable.Invoke<T>(object jsonObj)
    {
        if (jsonObj is JObject jobj)
        {
            Init(jobj);
            return (T)MessageRegistry.GetDeserializeFunction<T>()(this);
        }
        else
        {
            string json = (string)jsonObj;
            if (canUseJsonUtility)
                return JsonUtility.FromJson<T>(json);

            Init(json);
            return (T)MessageRegistry.GetDeserializeFunction<T>()(this);
        }
    }

    public T DeserializeMessage<T>(string json) where T : Message
    {
        if (canUseJsonUtility)
            return JsonUtility.FromJson<T>(json);

        Init(json);
        return (T)MessageRegistry.GetDeserializeFunction<T>()(this);
    }

    public T DeserializeMessage<T>(JObject jsonData) where T : Message
    {
        Init(jsonData);
        return (T)MessageRegistry.GetDeserializeFunction<T>()(this);
    }

    void Init(string json)
    {
        m_MessageStack.Clear();
        m_Root = JObject.Parse(json);
    }

    void Init(JObject jobject)
    {
        m_MessageStack.Clear();
        m_Root = jobject;
    }

    public void BeginMessage(string[] fieldNames)
    {
        JObject jsonData;
        jsonData = (m_MessageStack.Count == 0) ? m_Root : (JObject)ReadNextToken();
        Debug.Assert(jsonData.Count == fieldNames.Length); // all fields must be present
        JToken[] values = m_ValuesPool.GetOrCreate(fieldNames.Length);
        int index = 0;

        foreach (JProperty p in jsonData.Properties())
        {
            if (p.Name == fieldNames[index])
            {
                // most likely case: fields are serialized in order
                values[index] = p.Value;
            }
            else
            {
                // if that breaks down, look up the field by name
                int foundIndex = Array.IndexOf(fieldNames, p.Name);
                Debug.Assert(foundIndex >= 0 && values[foundIndex] == null);
                values[foundIndex] = p.Value;
            }
            index++;
        }
        m_MessageStack.Push(new MessageInProgress { values = values, numValues = fieldNames.Length });
    }

    JToken ReadNextToken()
    {
        MessageInProgress message = m_MessageStack.Pop();
        JToken result = message.nextValue;
        message.nextIndex++;
        m_MessageStack.Push(message);
        return result;
    }

    public void Read(out bool data) => data = (bool)ReadNextToken();
    public void Read(out bool[] data) => data = ((JArray)ReadNextToken()).ToObject<bool[]>();
    public void Read(out bool[] data, int fixedLength) => Read(out data);
    public void Read(out byte data) => data = (byte)ReadNextToken();
    public void Read(out byte[] data) => data = ((JArray)ReadNextToken()).ToObject<byte[]>();
    public void Read(out byte[] data, int fixedLength) => Read(out data);
    public void Read(out sbyte data) => data = (sbyte)ReadNextToken();
    public void Read(out sbyte[] data) => data = ((JArray)ReadNextToken()).ToObject<sbyte[]>();
    public void Read(out sbyte[] data, int fixedLength) => Read(out data);
    public void Read(out short data) => data = (short)ReadNextToken();
    public void Read(out short[] data) => data = ((JArray)ReadNextToken()).ToObject<short[]>();
    public void Read(out short[] data, int fixedLength) => Read(out data);
    public void Read(out ushort data) => data = (ushort)ReadNextToken();
    public void Read(out ushort[] data) => data = ((JArray)ReadNextToken()).ToObject<ushort[]>();
    public void Read(out ushort[] data, int fixedLength) => Read(out data);
    public void Read(out int data) => data = (int)ReadNextToken();
    public void Read(out int[] data) => data = ((JArray)ReadNextToken()).ToObject<int[]>();
    public void Read(out int[] data, int fixedLength) => Read(out data);
    public void Read(out uint data) => data = (uint)ReadNextToken();
    public void Read(out uint[] data) => data = ((JArray)ReadNextToken()).ToObject<uint[]>();
    public void Read(out uint[] data, int fixedLength) => Read(out data);
    public void Read(out long data) => data = (long)ReadNextToken();
    public void Read(out long[] data) => data = ((JArray)ReadNextToken()).ToObject<long[]>();
    public void Read(out long[] data, int fixedLength) => Read(out data);
    public void Read(out ulong data) => data = (ulong)ReadNextToken();
    public void Read(out ulong[] data) => data = ((JArray)ReadNextToken()).ToObject<ulong[]>();
    public void Read(out ulong[] data, int fixedLength) => Read(out data);
    public void Read(out string data) => data = (string)ReadNextToken();
    public void Read(out string[] data) => data = ((JArray)ReadNextToken()).ToObject<string[]>();
    public void Read(out string[] data, int fixedLength) => Read(out data);
    public void Read(out float data) => data = (float)ReadNextToken();
    public void Read(out float[] data) => data = ((JArray)ReadNextToken()).ToObject<float[]>();
    public void Read(out float[] data, int fixedLength) => Read(out data);
    public void Read(out double data) => data = (double)ReadNextToken();
    public void Read(out double[] data) => data = ((JArray)ReadNextToken()).ToObject<double[]>();
    public void Read(out double[] data, int fixedLength) => Read(out data);
    public void Read<T>(out T data) where T : Message => data = (T)MessageRegistry.GetDeserializeFunction<T>()(this);
    public void Read<T>(out T[] data) where T : Message
    {
        Func<IMessageDeserializer, Message> desFunc = MessageRegistry.GetDeserializeFunction<T>();
        JToken[] jtokens = ((JArray)ReadNextToken()).ToObject<JToken[]>();
        m_MessageStack.Push(new MessageInProgress { values = jtokens, nextIndex = 0 });

        data = new T[jtokens.Length];
        for (int Idx = 0; Idx < jtokens.Length; ++Idx)
        {
            data[Idx] = (T)desFunc(this);
            //data[Idx] = (T)MessageRegistry.GetDeserializeFunction<T>()(this);
        }

        m_MessageStack.Pop();
    }
    public void Read<T>(out T[] data, int fixedLength) where T : Message => Read(out data);

    public void EndMessage()
    {
        MessageInProgress oldMessage = m_MessageStack.Pop();
        Debug.Assert(oldMessage.nextIndex == oldMessage.numValues);
        m_ValuesPool.KeepValues(oldMessage);
    }

    struct JTokenPool
    {
        int m_MaxNormalFields;
        Stack<JToken[]> m_ValuesPool;

        public JTokenPool(int maxNormalFields)
        {
            m_MaxNormalFields = maxNormalFields;
            m_ValuesPool = new Stack<JToken[]>();
        }

        public JToken[] GetOrCreate(int length)
        {
            // if your message has a weirdly large number of fields, you don't get this optimization
            if (length > m_MaxNormalFields)
                return new JToken[length];

            if (m_ValuesPool.Count > 0)
                return m_ValuesPool.Pop();
            else
                return new JToken[m_MaxNormalFields];
        }

        public void KeepValues(MessageInProgress message)
        {
            if (message.numValues <= m_MaxNormalFields) // don't pool memory for messages with a weirdly large number of fields
                m_ValuesPool.Push(message.values);
        }
    }
    JTokenPool m_ValuesPool = new JTokenPool(16);
}
