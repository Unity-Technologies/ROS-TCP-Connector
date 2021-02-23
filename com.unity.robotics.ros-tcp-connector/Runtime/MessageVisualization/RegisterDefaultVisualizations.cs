using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RegisterDefaultVisualizations : MonoBehaviour
{
    void Awake()
    {
        foreach(IVisualizer c in gameObject.GetComponents<IVisualizer>())
        {
            c.Register(-1);
        }
    }
}
