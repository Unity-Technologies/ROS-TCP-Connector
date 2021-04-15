using RosMessageTypes.Visualization;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMarker : MonoBehaviour, IVisualizer, IPriority
{
    public int Priority { get; set; }
    Dictionary<string, Dictionary<int, BasicDrawing>> m_Drawings = new Dictionary<string, Dictionary<int, BasicDrawing>>();

    public virtual void Awake()
    {
        VisualizationRegister.RegisterVisualizer<MMarker>(this, Priority);
        VisualizationRegister.RegisterVisualizer<MMarkerArray>(this, Priority);
    }

    public object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing)
    {
        if (message is MMarkerArray)
        {
            foreach (MMarker marker in ((MMarkerArray)message).markers)
            {
                ProcessMarker(marker);
            }
            return true;
        }
        else if(message is MMarker)
        {
            ProcessMarker((MMarker)message);
            return true;
        }
        return null;
    }

    void ProcessMarker(MMarker marker)
    {
        Dictionary<int, BasicDrawing> ns;
        BasicDrawing drawing;
        switch (marker.action)
        {
            case MMarker.DELETEALL:
                foreach (Dictionary<int, BasicDrawing> namespaceToDestroy in m_Drawings.Values)
                    foreach(BasicDrawing drawingToDestroy in namespaceToDestroy.Values)
                        drawingToDestroy.Destroy();
                m_Drawings.Clear();
                break;
            case MMarker.ADD:
                if(!m_Drawings.TryGetValue(marker.ns, out ns))
                {
                    ns = new Dictionary<int, BasicDrawing>();
                    m_Drawings.Add(marker.ns, ns);
                }
                if(!ns.TryGetValue(marker.id, out drawing))
                {
                    drawing = BasicDrawing.Create();
                    ns.Add(marker.id, drawing);
                }
                if(marker.lifetime.secs == 0 && marker.lifetime.nsecs == 0)
                    drawing.ClearDuration();
                else
                    drawing.SetDuration(marker.lifetime.secs + marker.lifetime.nsecs/1E9f);
                drawing.Clear();
                marker.Draw<FLU>(drawing);
                break;
            case MMarker.DELETE:
                if (!m_Drawings.TryGetValue(marker.ns, out ns))
                {
                    ns = new Dictionary<int, BasicDrawing>();
                    m_Drawings.Add(marker.ns, ns);
                }
                if (ns.TryGetValue(marker.id, out drawing))
                {
                    drawing.Destroy();
                    ns.Remove(marker.id);
                }
                break;
        }
    }

    public Action CreateGUI(Message message, MessageMetadata meta, object drawing)
    {
        if (message is MMarkerArray)
        {
            return () =>
            {

            };
        }
        else if (message is MMarker)
        {
            return () =>
            {

            };
        }
        return null;
    }

    public void DeleteDrawing(object drawing)
    {
    }
}
