using RosMessageTypes.BuiltinInterfaces;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

// Represents a transform - position and rotation.
//(Like the Unity Transform class, but without the GameObject baggage that comes with it.)
public struct TFFrame
{
    public Vector3 translation;
    public Quaternion rotation;
    public static TFFrame identity = new TFFrame(Vector3.zero, Quaternion.identity);

    public TFFrame(Vector3 translation, Quaternion rotation)
    {
        this.translation = translation;
        this.rotation = rotation;
    }

    public Vector3 TransformPoint(Vector3 point)
    {
        return translation + rotation * point;
    }

    public Vector3 InverseTransformPoint(Vector3 point)
    {
        return Quaternion.Inverse(rotation) * (point - translation);
    }

    public TFFrame Compose(TFFrame child)
    {
        return new TFFrame(TransformPoint(child.translation), rotation * child.rotation);
    }

    public static TFFrame Lerp(TFFrame a, TFFrame b, float lerp)
    {
        return new TFFrame
        {
            translation = Vector3.Lerp(a.translation, b.translation, lerp),
            rotation = Quaternion.Lerp(a.rotation, b.rotation, lerp)
        };
    }
}

// Represents a transform frame changing over time.
public class TFStream
{
    public string Name { get; private set; }
    public string TFTopic { get; private set; }
    public TFStream Parent { get; private set; }
    public IEnumerable<TFStream> Children => m_Children;

    // oldest first
    List<long> m_Timestamps = new List<long>();
    // same order as m_Timestamps
    List<TFFrame> m_Frames = new List<TFFrame>();
    List<TFStream> m_Children = new List<TFStream>();

    // a gameobject at the last known position of this tfstream
    GameObject m_GameObject;
    public GameObject GameObject => m_GameObject;

    public TFStream(TFStream parent, string name, string tfTopic)
    {
        Name = name;
        TFTopic = tfTopic;
        m_GameObject = new GameObject(name);
        SetParent(parent);
    }

    public void SetParent(TFStream newParent)
    {
        if (Parent == newParent)
            return;

        if (Parent != null)
        {
            Parent.m_Children.Remove(this);
        }

        if (newParent != null)
        {
            m_GameObject.transform.parent = newParent.m_GameObject.transform;
            newParent.m_Children.Add(this);
        }
        else
        {
            m_GameObject.transform.parent = null;
        }
        Parent = newParent;
    }

    public void Add(long timestamp, Vector3 translation, Quaternion rotation)
    {
        TFFrame newEntry = new TFFrame(translation, rotation);
        // most likely case: we're just adding a newer transform to the end of the list
        if (m_Timestamps.Count == 0 || m_Timestamps[m_Timestamps.Count - 1] < timestamp)
        {
            m_Timestamps.Add(timestamp);
            m_Frames.Add(newEntry);
            m_GameObject.transform.localPosition = translation;
            m_GameObject.transform.localRotation = rotation;
        }
        else
        {
            int index = m_Timestamps.BinarySearch(timestamp);
            if (index < 0)
            {
                // no preexisting entry, but ~index gives us the position to insert the new entry
                m_Timestamps.Insert(~index, timestamp);
                m_Frames.Insert(~index, newEntry);
            }
            else
            {
                // we found an existing entry at the same timestamp!? Just replace the old one, I guess.
                m_Frames[index] = newEntry;
            }
        }

        // for now, just a lazy way to keep the buffer from growing infinitely: every 50 updates, discard the oldest 50
        if (m_Timestamps.Count > 100)
        {
            m_Timestamps.RemoveRange(0, 50);
            m_Frames.RemoveRange(0, 50);
        }
    }

    public TFFrame GetLocalTF(long time = 0)
    {
        // this stream has no data at all, so just report identity.
        if (m_Frames.Count == 0)
            return TFFrame.identity;

        // if time is 0, just get the newest position
        if (time == 0)
            return m_Frames[m_Frames.Count - 1];

        int index = m_Timestamps.BinarySearch(time);
        if (index >= 0)
        {
            // no problem, we have an entry at this time
            return m_Frames[index];
        }

        index = ~index;
        if (index == 0)
        {
            // older than our first entry: just use the first one
            return m_Frames[0];
        }
        else if (index == m_Frames.Count)
        {
            // newer than our last entry: just use the last one
            return m_Frames[m_Frames.Count - 1];
        }
        else
        {
            // between two entries: interpolate
            float lerpValue = (time - m_Timestamps[index - 1]) / (float)(m_Timestamps[index] - m_Timestamps[index - 1]);
            return TFFrame.Lerp(m_Frames[index - 1], m_Frames[index], lerpValue);
        }
    }

    public TFFrame GetLocalTF(TimeMsg time)
    {
        return GetLocalTF(time.ToLongTime());
    }

    public TFFrame GetWorldTF(long time = 0)
    {
        TFFrame parent;
        if (Parent != null)
            parent = Parent.GetWorldTF(time);
        else
            parent = TFFrame.identity;

        return parent.Compose(GetLocalTF(time));
    }

    public TFFrame GetWorldTF(TimeMsg time)
    {
        return GetWorldTF(time.ToLongTime());
    }

    // Can we safely stop polling for updates to a transform at this time?
    public bool IsTimeStable(long time)
    {
        if (time == 0) // time 0 ("use the newest data") is never stable
            return false;

        if (m_Timestamps.Count == 0 || m_Timestamps[0] > time || m_Timestamps[m_Timestamps.Count - 1] < time)
            return false;

        if (Parent != null && !Parent.IsTimeStable(time))
            return false;

        return true;
    }
}
