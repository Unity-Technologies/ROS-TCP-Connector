using System;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    public class TransformStreamFollower : MonoBehaviour
    {
        TransformStream m_Stream;
        // TODO: Implement a FollowMode here?

        public void OnValidate()
        {
            if (!Application.isPlaying)
            {
                Debug.LogWarning(
                    $"{nameof(TransformStreamFollower)} is expected to only be instantiated during PlayMode" +
                    " - are you doing something weird?");
            }
        }

        void Initialize(TransformStream target)
        {
            m_Stream = target;
            var thisParent = gameObject.transform.parent;
            if (m_Stream.Parent == null && thisParent != null)
            {
                Debug.LogWarning(
                    $"Following stream {m_Stream} which has no parent, but this object is parented to " +
                $"{thisParent.name}. This may result in undefined behavior.");
            }
            else if (m_Stream.Parent != null && m_Stream.Parent.Name != thisParent?.name)
            {
                Debug.LogWarning(
                    $"Following stream {m_Stream}, but the parent is named {m_Stream.Parent.Name} and this object is parented to "
                    + $"{thisParent?.name} which may result in undefined behavior.");
            }
        }

        public void Update()
        {
            if (m_Stream == null)
            {
                Debug.LogError($"{gameObject.name} has no stream to follow (this should be set immediately on creation) "
                    + $"disabling this {nameof(TransformStreamFollower)}");
                enabled = false;
                return;
            }

            var frame = m_Stream.GetLatestFrame();
            var tf = gameObject.transform;
            tf.localPosition = frame.Translation;
            tf.localRotation = frame.Rotation;
        }
    }
}
