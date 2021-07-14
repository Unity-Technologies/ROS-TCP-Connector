using System;
using System.Collections.Generic;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    public interface ITransformVisualizer
    {
        void OnChanged(TransformStream stream);
    }

    public class TransformManager
    {
        static ITransformVisualizer s_Visualizer;
        Dictionary<string, TransformStream> m_TransformTable = new Dictionary<string, TransformStream>();
        public static TransformManager instance { get; private set; }

        public static void Init()
        {
            if (instance == null)
            {
                instance = new TransformManager();

                // TODO: Add support for tf_static and a TransformStatic class?
                ROSConnection.instance.Subscribe<TFMessageMsg>("/tf", instance.Receive);
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
            if (visualizer == null)
            {
                Debug.LogWarning($"Tried to {nameof(Register)} a null {nameof(ITransformVisualizer)}");
            }

            s_Visualizer = visualizer;
            if (instance != null)
            {
                foreach (var stream in instance.m_TransformTable.Values)
                {
                    UpdateVisualization(stream);
                }
            }
            else
            {
                Debug.LogWarning($"Failed to update Visualization");
            }
        }

        public static void UpdateVisualization(TransformStream stream)
        {
            s_Visualizer?.OnChanged(stream);
        }

        public TransformFrame GetTransform(HeaderMsg header)
        {
            return GetTransform(header.frame_id, header.stamp.ToSeconds());
        }

        public static TransformFrame ComposeTfToBaseFrame(TransformStream stream, double time)
        {
            var frame = TransformFrame.Identity;
            while (stream.Parent != null)
            {
                frame = frame.Compose(stream.GetFrame(time));
                stream = stream.Parent;
            }

            return frame;
        }

        public TransformFrame GetTransform(string frameId, double time)
        {
            return m_TransformTable.TryGetValue(frameId, out var stream) ?
                ComposeTfToBaseFrame(stream, time) : TransformFrame.Identity;
        }

        public TransformFrame GetTransform(string frameId, TimeMsg time)
        {
            return GetTransform(frameId, time.ToSeconds());
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

        TransformStream GetOrCreateStream(string frameId, TransformStream parent)
        {
            // TODO: frameIds with no / prefix should get their namespace prepended
            frameId = frameId.Trim('/');

            var slash = frameId.LastIndexOf('/');
            var singleName = slash == -1 ? frameId : frameId.Substring(slash + 1);
            if (!m_TransformTable.TryGetValue(singleName, out var tf))
            {
                Debug.Log($"Creating new {nameof(TransformStream)} for {singleName} with parent {parent?.Name}");

                tf = new TransformStream(parent, singleName);

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

        void Receive(TFMessageMsg message)
        {
            foreach (var tfMessage in message.transforms)
            {
                var parentId = tfMessage.header.frame_id;
                var childId = tfMessage.child_frame_id;
                var streamParent = GetOrCreateStream(parentId, null);
                var streamChild = GetOrCreateStream(childId, streamParent);
                streamChild.Add(
                    tfMessage.header.stamp.ToSeconds(),
                    tfMessage.transform.translation.From<FLU>(),
                    tfMessage.transform.rotation.From<FLU>()
                );
                UpdateVisualization(streamChild);
            }
        }
    }
}
