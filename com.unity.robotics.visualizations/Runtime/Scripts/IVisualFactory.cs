using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public interface IVisualFactory
    {
        string Name { get; }
        string ID { get; }
        bool CanShowDrawing { get; }
        IVisual GetOrCreateVisual(string topic);
    }
}
