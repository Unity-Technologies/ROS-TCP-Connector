using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public interface IVisual
    {
        void SetDrawingEnabled(bool enabled);
        void CreateDrawing();
        void OnGUI();
    }
}
