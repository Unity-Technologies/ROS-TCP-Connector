using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Tf2;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    // TODO: The modification broadcast system is super generic, move to someplace more appropriate
    public class ModificationEventArgs<T> : EventArgs where T : class
    {

        public T ObjectChanged { get; }

        public ModificationEventArgs(T objectChanged)
        {
            ObjectChanged = objectChanged;
        }
    }

    public interface IModificationBroadcaster<T> where T : class
    {
        public event EventHandler<ModificationEventArgs<T>> OnChangeEvent;
        // Define with output in function signature to make it more tolerant of defining this interface more than
        // once on a single class
        public void ListTrackedObjects(out IEnumerable<T> trackedObjects);
    }

    public class TransformManager : IModificationBroadcaster<TransformStream>
    {
        Dictionary<string, TransformStream> m_TransformTable = new Dictionary<string, TransformStream>();
        public static TransformManager instance { get; private set; }

        public event EventHandler<ModificationEventArgs<TransformStream>> OnChangeEvent;

        public void ListTrackedObjects(out IEnumerable<TransformStream> trackedObjects)
        {
            trackedObjects = instance.m_TransformTable.Values;
        }

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

        void AnnounceChange(TransformStream stream)
        {
            // For thread safety, make a temporary copy
            var raiseEvent = OnChangeEvent;
            raiseEvent?.Invoke(this, new ModificationEventArgs<TransformStream>(stream));
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
                AnnounceChange(tf);
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
                if (streamParent == null)
                {
                    throw new InvalidOperationException($"Failed to create the parent stream, {parentId}."
                        + $"The child stream {childId} will also fail.");
                }

                var streamChild = GetOrCreateStream(childId, streamParent);
                streamChild.AddFrame(
                    tfMessage.header.stamp.ToSeconds(),
                    tfMessage.transform.translation.From<FLU>(),
                    tfMessage.transform.rotation.From<FLU>(),
                    streamParent
                );
                AnnounceChange(streamChild);
            }
        }
    }
}
