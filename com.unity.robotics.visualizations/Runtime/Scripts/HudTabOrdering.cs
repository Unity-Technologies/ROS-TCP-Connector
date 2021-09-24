using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    // By default a user-created tab is assigned a positive index.
    // The built in tabs have a negative index so that they always come first.
    public enum HudTabOrdering
    {
        Topics = -4,
        TF = -3,
        Layout = -2,
        Markers = -1,
    }
}
