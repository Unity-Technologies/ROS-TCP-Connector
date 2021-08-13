using RosMessageTypes.Std;
using RosMessageTypes.Visualization;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class DefaultVisualizerMarker : MonoBehaviour, IVisualFactory, IPriority
{
    public int Priority { get; set; }
    Dictionary<string, Dictionary<int, BasicDrawing>> m_Drawings = new Dictionary<string, Dictionary<int, BasicDrawing>>();
    public bool CanShowDrawing => true;

    public virtual void Awake()
    {
        VisualFactoryRegistry.RegisterTypeVisualizer<MarkerMsg>(this, Priority);
        VisualFactoryRegistry.RegisterTypeVisualizer<MarkerArrayMsg>(this, Priority);
    }

    public IVisual CreateVisual()
    {
        // TODO
        return null;
    }

    public object CreateDrawing(Message message, MessageMetadata meta, object oldDrawing)
    {
        if (message is MarkerArrayMsg)
        {
            foreach (MarkerMsg marker in ((MarkerArrayMsg)message).markers)
            {
                ProcessMarker(marker);
            }
            return true;
        }
        else if (message is MarkerMsg)
        {
            ProcessMarker((MarkerMsg)message);
            return true;
        }
        return null;
    }

    void ProcessMarker(MarkerMsg marker)
    {
        Dictionary<int, BasicDrawing> ns;
        BasicDrawing drawing;
        switch (marker.action)
        {
            case MarkerMsg.DELETEALL:
                foreach (Dictionary<int, BasicDrawing> namespaceToDestroy in m_Drawings.Values)
                    foreach (BasicDrawing drawingToDestroy in namespaceToDestroy.Values)
                        drawingToDestroy.Destroy();
                m_Drawings.Clear();
                break;
            case MarkerMsg.ADD:
                if (!m_Drawings.TryGetValue(marker.ns, out ns))
                {
                    ns = new Dictionary<int, BasicDrawing>();
                    m_Drawings.Add(marker.ns, ns);
                }
                if (!ns.TryGetValue(marker.id, out drawing))
                {
                    drawing = BasicDrawing.Create();
                    ns.Add(marker.id, drawing);
                }
                if (marker.lifetime.sec == 0 && marker.lifetime.nanosec == 0)
                    drawing.ClearDuration();
                else
                    drawing.SetDuration(marker.lifetime.sec + marker.lifetime.nanosec / 1E9f);
                drawing.Clear();
                marker.Draw<FLU>(drawing);
                break;
            case MarkerMsg.DELETE:
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
        if (message is MarkerArrayMsg)
        {
            return () =>
            {

            };
        }
        else if (message is MarkerMsg)
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

    public HeaderMsg GetHeader(Message message)
    {
        return null;
    }
}
