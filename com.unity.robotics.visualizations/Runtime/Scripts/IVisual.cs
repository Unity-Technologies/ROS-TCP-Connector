using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public interface IVisual
    {
        bool IsDrawingEnabled { get; }
        void SetDrawingEnabled(bool enabled);
        void Redraw();
        void OnGUI();
    }
}
