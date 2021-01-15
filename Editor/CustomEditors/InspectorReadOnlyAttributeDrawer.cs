using UnityEditor;
using UnityEngine;

namespace RosEditorUtilities
{
    [CustomPropertyDrawer(typeof(InspectorReadOnlyAttribute))]
    public class InspectorReadOnlyAttributeDrawer : PropertyDrawer
    {
        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            return isHidden? 0: EditorGUI.GetPropertyHeight(property, label, true);
        }

        public bool isHidden => (EditorApplication.isPlaying ?
            ((InspectorReadOnlyAttribute)attribute).hideInPlayMode :
            ((InspectorReadOnlyAttribute)attribute).hideInEditMode);

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            if (!isHidden)
            {
                EditorGUI.LabelField(position, property.displayName + ": " + property.stringValue, EditorStyles.boldLabel);
            }
        }
    } 
}