using UnityEngine;

public class InspectorReadOnlyAttribute : PropertyAttribute
{
    public bool hideInPlayMode;
    public bool hideInEditMode;

    public InspectorReadOnlyAttribute(bool hideInEditMode = false, bool hideInPlayMode = false)
    {
        this.hideInEditMode = hideInEditMode;
        this.hideInPlayMode = hideInPlayMode;
    }
} 