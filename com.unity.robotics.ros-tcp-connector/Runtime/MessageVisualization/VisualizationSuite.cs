using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VisualizationSuite : MonoBehaviour
{
    public int priority;

    void Awake()
    {
        foreach(IVisualizerConfig c in gameObject.GetComponents<IVisualizerConfig>())
        {
            c.Register(priority);
        }
    }
}
