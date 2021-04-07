using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.MessageVisualizers;

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

// Represents a hierarchy of transforms changing over time.
public class TFStream
{
    // in ascending order of time
    List<long> m_Timestamps = new List<long>();
    // same order as m_Times
    List<TFFrame> m_Frames = new List<TFFrame>();
    TFStream m_Parent;
    GameObject m_GameObject;

    public TFStream(TFStream parent)
    {
        m_Parent = parent;
    }

    public void Add(long timestamp, Vector3 translation, Quaternion rotation)
    {
        TFFrame newEntry = new TFFrame( translation, rotation );
        // most likely case: we're just adding a newer transform to the end of the list
        if (m_Timestamps.Count == 0 || m_Timestamps[m_Timestamps.Count - 1] < timestamp)
        {
            m_Timestamps.Add(timestamp);
            m_Frames.Add(newEntry);
        }
        else
        {
            int index = m_Timestamps.BinarySearch(timestamp);
            if(index < 0)
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

        // for now, just a lazy way to keep the buffer from growing infinitely: every 100 frames, discard the oldest 50
        if (m_Timestamps.Count > 100)
        {
            m_Timestamps.RemoveRange(0, 50);
            m_Frames.RemoveRange(0, 50);
        }
    }

    public TFFrame GetLocalTF(long time)
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

    public TFFrame GetWorldTF(long time)
    {
        TFFrame parent;
        if (m_Parent != null)
            parent = m_Parent.GetWorldTF(time);
        else
            parent = TFFrame.identity;

        return parent.Compose(GetLocalTF(time));
    }

    // Can we safely stop polling for updates to a transform at this time?
    public bool IsTimeStable(long time)
    {
        if (time == 0) // time 0 ("use the newest data") is never stable
            return false;

        if (m_Timestamps.Count == 0 || m_Timestamps[0] > time || m_Timestamps[m_Timestamps.Count - 1] < time)
            return false;

        if (m_Parent != null && !m_Parent.IsTimeStable(time))
            return false;

        return true;
    }

    public void ShowWidget(string name)
    {
        if (m_GameObject == null)
            m_GameObject = new GameObject(name);
        TFFrame frame = GetWorldTF(0);
        m_GameObject.transform.position = frame.translation;
        m_GameObject.transform.rotation = frame.rotation;
    }
}
