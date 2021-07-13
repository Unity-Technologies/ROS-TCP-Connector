using System;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    public interface ITransformVisualizer
    {
        void OnChanged(TransformStream stream);
    }

    public class TransformGraph
    {
        static ITransformVisualizer s_Visualizer;
        Dictionary<string, TransformStream> m_TransformTable = new Dictionary<string, TransformStream>();
        Dictionary<string, Transform> m_TrackingTransformTable = new Dictionary<string, Transform>();
        public static TransformGraph instance { get; private set; }

        public static void Init()
        {
            if (instance == null)
            {
                instance = new TransformGraph();

                // TODO: Add support for tf_static and a TransformStatic class?
                ROSConnection.instance.Subscribe<TFMessageMsg>("/tf", instance.ReceiveTF);
            }
        }

        public IEnumerable<string> GetTransformNames()
        {
            return m_TransformTable.Keys;
        }

        public IEnumerable<TransformStream> GetTransforms()
        {
            return m_TransformTable.Values;
        }

        public static void Register(ITransformVisualizer visualizer)
        {
            s_Visualizer = visualizer;
            if (instance != null)
                foreach (var stream in instance.m_TransformTable.Values)
                    UpdateVisualization(stream);
        }

        public static void UpdateVisualization(TransformStream stream)
        {
            s_Visualizer?.OnChanged(stream);
        }

        public TransformFrame GetTransform(HeaderMsg header)
        {
            return GetTransform(header.frame_id, header.stamp.ToLongTime());
        }

        public static TransformFrame ComposeTfToBaseFrame(TransformStream stream, long time)
        {
            var frame = TransformFrame.Identity;
            while (stream.Parent != null)
            {
                frame = frame.Compose(stream.GetFrame(time));
                stream = stream.Parent;
            }

            return frame;
        }

        public TransformFrame GetTransform(string frameId, long time)
        {
            return m_TransformTable.TryGetValue(frameId, out var stream) ?
                ComposeTfToBaseFrame(stream, time) : TransformFrame.Identity;
        }

        public TransformFrame GetTransform(string frameId, TimeMsg time)
        {
            return GetTransform(frameId, time.ToLongTime());
        }

        public TransformStream GetTransformStream(string frameId)
        {
            if (m_TransformTable.TryGetValue(frameId, out var stream))
            {
                return stream;
            }

            throw new ArgumentException($"Failed to find {frameId} in {nameof(m_TransformTable)}.");
        }

        public GameObject GetTransformGameObject(string frameId)
        {
            return GetTransformStream(frameId).GameObject;
        }

        TransformStream GetOrCreateStream(string frameId)
        {
            // TODO: frameIds with no / prefix should get their namespace prepended
            frameId = frameId.Trim('/');

            var slash = frameId.LastIndexOf('/');
            var singleName = slash == -1 ? frameId : frameId.Substring(slash + 1);
            if (!m_TransformTable.TryGetValue(singleName, out var tf))
            {
                if (slash < 0)
                {
                    tf = new TransformStream(null, singleName);
                }
                else
                {
                    var parent = GetOrCreateStream(frameId.Substring(0, slash));
                    tf = new TransformStream(parent, singleName);
                }

                m_TransformTable[singleName] = tf;
                UpdateVisualization(tf);
            }
            else if (tf == null)
            {
                throw new InvalidOperationException(
                    $"{nameof(m_TransformTable)} had a null {nameof(TransformStream)} for key {singleName}. What went wrong?");
            }

            return tf;
        }

        void ReceiveTF(TFMessageMsg message)
        {
            foreach (var tfMessage in message.transforms)
            {
                // TODO: >>> FIX: Remove this concatenation and re-work stream creation logic to remove assumption
                // TODO:           that string formatting implies tf relationship, do proper graph search for
                // TODO:           existing relationship and include handling for error cases like pre-existing
                // TODO:           child parented to different object
                var frameId = tfMessage.header.frame_id + "/" + tfMessage.child_frame_id;
                var tf = GetOrCreateStream(frameId);
                tf.Add(
                    tfMessage.header.stamp.ToLongTime(),
                    tfMessage.transform.translation.From<FLU>(),
                    tfMessage.transform.rotation.From<FLU>()
                );
                UpdateVisualization(tf);
            }
        }
    }
}
