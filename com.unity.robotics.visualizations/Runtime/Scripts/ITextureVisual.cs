using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public interface ITextureVisual : IVisual
    {
        Texture2D GetTexture();
        void ListenForTextureChange(Action<Texture2D> callback);
    }
}
