using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public interface ITFSystemVisualizer
{
    void OnChanged(TFStream stream);
}

public class TFSystem
{
    Dictionary<string, TFStream> m_TransformTable = new Dictionary<string, TFStream>();
    public static TFSystem instance { get; private set; }

    // Default visualization settings
    public bool ShowTFAxesDefault { get; set; }
    public bool ShowTFLinksDefault { get; set; }
    public bool ShowTFNamesDefault { get; set; }

    public static void Init()
    {
        if (instance == null)
        {
            instance = new TFSystem();
            ROSConnection.instance.RegisterSubscriber("/tf", MTFMessage.k_RosMessageName);
            ROSConnection.instance.Subscribe<MTFMessage>("/tf", instance.ReceiveTF);
        }
    }

    public IEnumerable<string> GetTransformNames() => m_TransformTable.Keys;
    public IEnumerable<TFStream> GetTransforms() => m_TransformTable.Values;

    static ITFSystemVisualizer s_Visualizer = null;

    public static void Register(ITFSystemVisualizer visualizer)
    {
        s_Visualizer = visualizer;
        if (instance != null)
        {
            foreach (TFStream stream in instance.m_TransformTable.Values)
                UpdateVisualization(stream);
        }
    }

    public static void UpdateVisualization(TFStream stream)
    {
        if(s_Visualizer != null)
            s_Visualizer.OnChanged(stream);
    }

    public TFFrame GetTransform(MHeader header)
    {
        return GetTransform(header.frame_id, header.stamp.ToLongTime());
    }

    public TFFrame GetTransform(string frame_id, long time)
    {
        TFStream stream = GetTransformStream(frame_id);
        if (stream != null)
            return stream.GetWorldTF(time);
        else
            return TFFrame.identity;
    }

    public TFFrame GetTransform(string frame_id, MTime time)
    {
        return GetTransform(frame_id, time.ToLongTime());
    }

    public TFStream GetTransformStream(string frame_id)
    {
        TFStream tf;
        m_TransformTable.TryGetValue(frame_id, out tf);
        return tf;
    }

    TFStream GetOrCreateTFStream(string frame_id)
    {
        TFStream tf;
        while (frame_id.EndsWith("/"))
            frame_id = frame_id.Substring(0, frame_id.Length - 1);

        int slash = frame_id.LastIndexOf('/');
        string singleName = slash == -1 ? frame_id : frame_id.Substring(slash + 1);
        if (!m_TransformTable.TryGetValue(singleName, out tf) || tf == null)
        {
            if (slash <= 0)
            {
                // there's no slash, or only an initial slash - just create a new root object
                tf = new TFStream(null, singleName);
            }
            else
            {
                TFStream parent = GetOrCreateTFStream(frame_id.Substring(0, slash));
                tf = new TFStream(parent, singleName);
            }
            tf.ShowAxes = ShowTFAxesDefault;
            tf.ShowLink = ShowTFLinksDefault;
            tf.ShowName = ShowTFNamesDefault;
            m_TransformTable[singleName] = tf;
            UpdateVisualization(tf);
        }
        return tf;
    }

    void ReceiveTF(MTFMessage message)
    {
        foreach (MTransformStamped tf_message in message.transforms)
        {
            string frame_id = tf_message.header.frame_id + "/" + tf_message.child_frame_id;
            TFStream tf = GetOrCreateTFStream(frame_id);
            tf.Add(
                tf_message.header.stamp.ToLongTime(),
                tf_message.transform.translation.From<FLU>(),
                tf_message.transform.rotation.From<FLU>()
            );
            UpdateVisualization(tf);
        }
    }
}
