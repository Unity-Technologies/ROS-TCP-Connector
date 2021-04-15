using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class TFSystemVisualizer : MonoBehaviour, ITFSystemVisualizer
    {
        public float axesScale = 0.1f;
        public float lineThickness = 0.01f;
        public Color color = Color.white;
        Dictionary<string, BasicDrawing> drawings = new Dictionary<string, BasicDrawing>();

        public void Start()
        {
            TFSystem.Register(this);
            if (color.a == 0)
                color.a = 1;
        }

        public void OnChanged(TFStream stream)
        {
            BasicDrawing drawing;
            if(!drawings.TryGetValue(stream.Name, out drawing))
            {
                drawing = BasicDrawing.Create();
                drawings[stream.Name] = drawing;
                if (stream.Parent != null)
                {
                    OnChanged(stream.Parent);
                    BasicDrawing parentStream;
                    if (drawings.TryGetValue(stream.Parent.Name, out parentStream))
                    {
                        drawing.transform.parent = parentStream.transform;
                    }
                }
            }

            TFFrame frame = stream.GetLocalTF();

            drawing.transform.localPosition = frame.translation;
            drawing.transform.localRotation = frame.rotation;
            drawing.Clear();
            if (stream.ShowAxes)
                MessageVisualizations.DrawAxisVectors<FLU>(drawing, Vector3.zero.To<FLU>(), Quaternion.identity.To<FLU>(), axesScale, false);

            if (stream.ShowLink)
                drawing.DrawLine(Quaternion.Inverse(frame.rotation) * -frame.translation, Vector3.zero, color, lineThickness);

            if (stream.ShowName)
                drawing.DrawLabel(stream.Name, Vector3.zero, color);
        }
    }
}