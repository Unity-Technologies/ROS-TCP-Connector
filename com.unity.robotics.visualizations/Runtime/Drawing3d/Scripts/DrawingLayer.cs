using System;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Unity.Robotics.Visualizations
{
    [Serializable]
    public class DrawingLayer
    {
        public int LayerNumber = 5; // Default to builtin UI layer
    }

#if UNITY_EDITOR
    [CustomPropertyDrawer(typeof(DrawingLayer))]
    public class DrawingLayerPropertyDrawer : PropertyDrawer 
    {
        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, GUIContent.none, property);
            SerializedProperty layer = property.FindPropertyRelative("LayerNumber");
            position = EditorGUI.PrefixLabel(position, GUIUtility.GetControlID(FocusType.Passive), label);
            if (layer != null)
                layer.intValue = EditorGUI.LayerField(position, layer.intValue);
            EditorGUI.EndProperty();
        }
    }
#endif
}