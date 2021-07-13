using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;
using UnityEditorInternal;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    // Represents a transform frame changing over time.
    public class TransformStream
    {
        const double k_HistoryMaxLengthSeconds = 10.0;
        const int k_HistoryMaxNumEntries = 100;

        SortedList<long, TransformFrame> m_Transforms = new SortedList<long, TransformFrame>(k_HistoryMaxNumEntries);

        HashSet<TransformStream> m_Children = new HashSet<TransformStream>();

        // a gameobject at the last known position of this tfstream
        GameObject m_GameObject;

        public string Name { get; }

        public TransformStream Parent { get; }

        public IEnumerable<TransformStream> Children => m_Children;

        public GameObject GameObject => m_GameObject;

        public TransformStream(TransformStream parent, string name)
        {
            Parent = parent;
            Name = name;
            m_GameObject = new GameObject(name);
            if (parent != null)
            {
                m_GameObject.transform.parent = parent.m_GameObject.transform;
                parent.AddChildStream(this);
            }
        }

        void AddChildStream(TransformStream child)
        {
            m_Children.Add(child);
        }

        public void Add(long timestamp, Vector3 translation, Quaternion rotation)
        {
            if (Parent == null)
            {
                // Can't add transforms for root nodes (nodes with no parent to transform to)
                throw new InvalidOperationException(
                    $"Cannot add {nameof(TransformFrame)} to {Name} because you must assign a {nameof(Parent)} first.");
            }

            var newEntry = new TransformFrame(translation, rotation);
            if (m_Transforms.ContainsKey(timestamp))
            {
                // we found an existing entry at the same timestamp!? Just replace the old one, I guess.
                Debug.LogWarning(
                    $"Transform {newEntry} has same timestamp as {m_Transforms[timestamp]} this will be overwritten.");
            }

            // Only need to check if we're at the max entry limit if the entry is being added and not overwritten
            else
            {
                if (m_Transforms.Count == k_HistoryMaxNumEntries)
                {
                    m_Transforms.RemoveAt(0);
                }
            }

            m_Transforms.Add(timestamp, newEntry);

            var latestEntry = m_Transforms.Last();

            while (latestEntry.Key - m_Transforms.First().Key > k_HistoryMaxLengthSeconds)
            {
                m_Transforms.RemoveAt(0);
            }

            var latestFrame = latestEntry.Value;
            m_GameObject.transform.SetPositionAndRotation(latestFrame.Translation, latestFrame.Rotation);
        }

        TransformFrame GetFrameFailSilently(long time = 0)
        {
            // this stream has no data at all, so just report identity.
            if (m_Transforms.Count == 0)
                return TransformFrame.Identity;

            if (time == 0)
                return m_Transforms.Last().Value;

            if (m_Transforms.TryGetValue(time, out var frame))
            {
                return frame;
            }

            var index = ~BinarySearchForIndex(time);

            if (index == 0)
            {
                return m_Transforms.First().Value;
            }

            if (index == m_Transforms.Count)
            {
                return m_Transforms.Last().Value;
            }

            return Interpolate(time, index);
        }

        public bool TryGetFrame(long time, out TransformFrame frame, out string failureReason)
        {
            failureReason = "";

            if (m_Transforms.Count == 0)
            {
                failureReason = $"has no {nameof(TransformFrame)}s in {nameof(m_Transforms)}";
            }
            else if (time < m_Transforms.First().Key)
            {
                failureReason = $"time requested was earlier than earliest time available: {m_Transforms.First().Key:F}";
            }
            else if (time > m_Transforms.Last().Key)
            {
                failureReason = $"time requested was later than latest time available: {m_Transforms.Last().Key:F}";
            }
            else if (m_Transforms.TryGetValue(time, out frame))
            {
                return true;
            }
            else
            {
                frame = Interpolate(time, ~BinarySearchForIndex(time));
                return true;
            }

            frame = TransformFrame.Identity;
            return false;
        }

        public TransformFrame GetFrame(long time, bool shouldFailSilently = false)
        {
            if (shouldFailSilently)
            {
                return GetFrameFailSilently(time);
            }

            if (TryGetFrame(time, out var frame, out var failureReason))
            {
                return frame;
            }

            throw new InvalidOperationException(
                $"{nameof(TransformStream)} {Name} failed to get a {nameof(TransformFrame)} to {Parent.Name} at time {time} because {failureReason}.");
        }

        TransformFrame Interpolate(long time, int endIndex)
        {
            var start = m_Transforms.ElementAt(endIndex - 1);
            var end = m_Transforms.ElementAt(endIndex);
            var t = (time - start.Key) / (end.Key - start.Key);
            return TransformFrame.Lerp(start.Value, end.Value, t);
        }

        int BinarySearchForIndex(long time)
        {
            return ((List<long>)m_Transforms.Keys).BinarySearch(time);
        }
    }
}
