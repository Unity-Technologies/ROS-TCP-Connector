using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    // Represents a transform frame changing over time.
    class TransformStream
    {
        const double k_HistoryMaxLengthSeconds = 10.0;
        const int k_HistoryMaxNumEntries = 100;

        TransformStream m_Parent;
        protected SortedList<double, TransformFrame> m_Transforms =
            new SortedList<double, TransformFrame>(k_HistoryMaxNumEntries);

        HashSet<TransformStream> m_Children = new HashSet<TransformStream>();

        public string Name { get; }

        public TransformStream Parent
        {
            get => m_Parent;
            protected set
            {
                if (m_Parent != null)
                {
                    Debug.LogWarning($"{nameof(m_Parent)} has already been assigned as {m_Parent.Name}. "
                    + $"Re-assigning to {value?.Name} which could lead to undefined behavior.");
                    m_Parent.RemoveChildStream(this);
                }
                else
                {
                    Debug.LogWarning($"{Name} was created without a parent transform - is this a global frame?");
                }

                value?.AddChildStream(this);

                m_Parent = value;
            }
        }

        public IEnumerable<TransformStream> Children => m_Children;


        public TransformStream(TransformStream parent, string name)
        {
            Name = name;
            Parent = parent;
        }

        void AddChildStream(TransformStream child)
        {
            m_Children.Add(child);
        }

        void RemoveChildStream(TransformStream child)
        {
            if (!m_Children.Contains(child))
            {
                throw new InvalidOperationException(
                    $"Can't remove {child?.Name} from {nameof(m_Children)} because it is not in the set.");
            }

            m_Children.Remove(child);
        }

        internal void AddFrame(double timestamp, Vector3 translation, Quaternion rotation,
            TransformStream parentStream = null)
        {
            if (Parent == null)
            {
                if (parentStream != null)
                {
                    Debug.Log($"Transform received for orphan node {Name}, setting {parentStream.Name} as the parent.");
                    Parent = parentStream;
                }
                else
                {
                    // Can't add transforms for root nodes (nodes with no parent to transform to)
                    throw new InvalidOperationException(
                        $"Cannot add {nameof(TransformFrame)} to {Name} because you must assign a {nameof(Parent)} first.");
                }
            }

            var newEntry = new TransformFrame(translation, rotation);
            if (m_Transforms.ContainsKey(timestamp))
            {
                // we found an existing entry at the same timestamp!? Just replace the old one, I guess.
                Debug.LogWarning(
                    $"Transform {newEntry} has same timestamp ({timestamp}) as {m_Transforms[timestamp]} for {this}, "
                            + "the entry will be overwritten.");
                m_Transforms.Remove(timestamp);
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
        }

        internal TransformFrame GetLatestFrame()
        {
            return m_Transforms.Values.Last();
        }

        TransformFrame GetFrameFailSilently(double time)
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

        internal bool TryGetFrame(double time, out TransformFrame frame, out string failureReason)
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

        internal TransformFrame GetFrame(double time, bool shouldFailSilently = false)
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

        TransformFrame Interpolate(double time, int endIndex)
        {
            var start = m_Transforms.ElementAt(endIndex - 1);
            var end = m_Transforms.ElementAt(endIndex);
            var t = (float)((time - start.Key) / (end.Key - start.Key));
            return TransformFrame.Lerp(start.Value, end.Value, t);
        }

        int BinarySearchForIndex(double time)
        {
            return ((List<double>)m_Transforms.Keys).BinarySearch(time);
        }

        public override string ToString()
        {
            return $"{Name}({nameof(TransformStream)})";
        }
    }
}
