using System;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class TFSystem
{
    public static TFSystem instance { get; private set; }
    Dictionary<string, TFTopicState> m_TFTopics = new Dictionary<string, TFTopicState>();

    public class TFTopicState
    {
        string m_TFTopic;
        Dictionary<string, TFStream> m_TransformTable = new Dictionary<string, TFStream>();
        List<Action<TFStream>> m_Listeners = new List<Action<TFStream>>();

        public TFTopicState(string tfTopic = "/tf")
        {
            m_TFTopic = tfTopic;
            ROSConnection.GetOrCreateInstance().Subscribe<TFMessageMsg>(tfTopic, ReceiveTF);
        }

        public TFStream GetOrCreateFrame(string frame_id)
        {
            TFStream tf;
            while (frame_id.EndsWith("/"))
                frame_id = frame_id.Substring(0, frame_id.Length - 1);

            var slash = frame_id.LastIndexOf('/');
            var singleName = slash == -1 ? frame_id : frame_id.Substring(slash + 1);
            if (!m_TransformTable.TryGetValue(singleName, out tf) || tf == null)
            {
                if (slash <= 0)
                {
                    // there's no slash, or only an initial slash - just create a new root object
                    // (set the parent later if and when we learn more)
                    tf = new TFStream(null, singleName, m_TFTopic);
                }
                else
                {
                    var parent = GetOrCreateFrame(frame_id.Substring(0, slash));
                    tf = new TFStream(parent, singleName, m_TFTopic);
                }

                m_TransformTable[singleName] = tf;
                NotifyChanged(tf);
            }
            else if (slash > 0 && tf.Parent == null)
            {
                tf.SetParent(GetOrCreateFrame(frame_id.Substring(0, slash)));
            }

            return tf;
        }

        public void ReceiveTF(TFMessageMsg message)
        {
            foreach (var tf_message in message.transforms)
            {
                var frame_id = tf_message.header.frame_id + "/" + tf_message.child_frame_id;
                var tf = GetOrCreateFrame(frame_id);
                tf.Add(
                    tf_message.header.stamp.ToLongTime(),
                    tf_message.transform.translation.From<FLU>(),
                    tf_message.transform.rotation.From<FLU>()
                );
                NotifyChanged(tf);
            }
        }

        public IEnumerable<string> GetTransformNames()
        {
            return m_TransformTable.Keys;
        }

        public IEnumerable<TFStream> GetTransforms()
        {
            return m_TransformTable.Values;
        }

        public TFStream GetTransformStream(string frame_id)
        {
            TFStream result = null;
            m_TransformTable.TryGetValue(frame_id, out result);
            return result;
        }

        public void AddListener(Action<TFStream> callback)
        {
            m_Listeners.Add(callback);
        }

        public void NotifyChanged(TFStream stream)
        {
            foreach (Action<TFStream> callback in m_Listeners)
            {
                callback(stream);
            }
        }

        public void NotifyAllChanged()
        {
            foreach (var stream in m_TransformTable.Values)
                NotifyChanged(stream);
        }
    }

    private TFSystem()
    {

    }

    public static TFSystem GetOrCreateInstance()
    {
        if (instance != null)
            return instance;

        ROSConnection ros = ROSConnection.GetOrCreateInstance();
        instance = new TFSystem();
        foreach (string s in ros.TFTopics)
        {
            instance.GetOrCreateTFTopic(s);
        }
        return instance;
    }

    public IEnumerable<string> GetTransformNames(string tfTopic = "/tf")
    {
        return GetOrCreateTFTopic(tfTopic).GetTransformNames();
    }

    public IEnumerable<TFStream> GetTransforms(string tfTopic = "/tf")
    {
        return GetOrCreateTFTopic(tfTopic).GetTransforms();
    }

    public void AddListener(Action<TFStream> callback, bool notifyAllStreamsNow = true, string tfTopic = "/tf")
    {
        TFTopicState state = GetOrCreateTFTopic(tfTopic);
        state.AddListener(callback);
        if (notifyAllStreamsNow)
            state.NotifyAllChanged();
    }

    public void NotifyAllChanged(TFStream stream)
    {
        GetOrCreateTFTopic(stream.TFTopic).NotifyAllChanged();
    }

    public TFFrame GetTransform(HeaderMsg header, string tfTopic = "/tf")
    {
        return GetTransform(header.frame_id, header.stamp.ToLongTime(), tfTopic);
    }

    public TFFrame GetTransform(string frame_id, long time, string tfTopic = "/tf")
    {
        var stream = GetTransformStream(frame_id, tfTopic);
        if (stream != null)
            return stream.GetWorldTF(time);
        return TFFrame.identity;
    }

    public TFFrame GetTransform(string frame_id, TimeMsg time, string tfTopic = "/tf")
    {
        return GetTransform(frame_id, time.ToLongTime(), tfTopic);
    }

    public TFStream GetTransformStream(string frame_id, string tfTopic = "/tf")
    {
        return GetOrCreateTFTopic(tfTopic).GetTransformStream(frame_id);
    }

    public GameObject GetTransformObject(string frame_id, string tfTopic = "/tf")
    {
        TFStream stream = GetOrCreateTFTopic(tfTopic).GetOrCreateFrame(frame_id);
        return stream.GameObject;
    }

    public TFTopicState GetOrCreateTFTopic(string tfTopic = "/tf")
    {
        TFTopicState tfTopicState;
        if (!m_TFTopics.TryGetValue(tfTopic, out tfTopicState))
        {
            tfTopicState = new TFTopicState(tfTopic);
            m_TFTopics[tfTopic] = tfTopicState;
        }
        return tfTopicState;
    }

    public TFStream GetOrCreateFrame(string frame_id, string tfTopic = "/tf")
    {
        TFTopicState topicState = GetOrCreateTFTopic(tfTopic);
        return topicState.GetOrCreateFrame(frame_id);
    }
}
