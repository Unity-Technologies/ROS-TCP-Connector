using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class TFSystemVisualizer : MonoBehaviour, ITFSystemVisualizer, ROSTCPConnector.IHudTab
    {
        public float axesScale = 0.1f;
        public float lineThickness = 0.01f;
        public Color color = Color.white;
        Dictionary<string, BasicDrawing> drawings = new Dictionary<string, BasicDrawing>();

        public void Start()
        {
            HUDPanel.RegisterTab(this);
            TFSystem.Register(this);
            if (color.a == 0)
                color.a = 1;
        }

        void EnsureSettings(TFStream stream)
        {
            if (!m_ShowExpanded.ContainsKey(stream))
            {
                m_ShowExpanded.Add(stream, true);
                m_ShowAxes.Add(stream, ShowTFAxesDefault);
                m_ShowLinks.Add(stream, ShowTFLinksDefault);
                m_ShowNames.Add(stream, ShowTFNamesDefault);
            }
        }

        public void OnChanged(TFStream stream)
        {
            BasicDrawing drawing;
            if (!drawings.TryGetValue(stream.Name, out drawing))
            {
                drawing = BasicDrawing.Create();
                drawings[stream.Name] = drawing;
                if (stream.Parent != null)
                {
                    OnChanged(stream.Parent);
                    BasicDrawing parentStream;
                    if (drawings.TryGetValue(stream.Parent.Name, out parentStream))
                    {
                        drawing.transform.parent = parentStream.transform;
                    }
                }
            }

            TFFrame frame = stream.GetLocalTF();

            drawing.transform.localPosition = frame.translation;
            drawing.transform.localRotation = frame.rotation;
            drawing.Clear();
            EnsureSettings(stream);
            if (m_ShowAxes[stream])
                MessageVisualizations.DrawAxisVectors<FLU>(drawing, Vector3.zero.To<FLU>(), Quaternion.identity.To<FLU>(), axesScale, false);

            if (m_ShowLinks[stream])
                drawing.DrawLine(Quaternion.Inverse(frame.rotation) * -frame.translation, Vector3.zero, color, lineThickness);

            if (m_ShowNames[stream])
                drawing.DrawLabel(stream.Name, Vector3.zero, color);
        }

        // === Code for the "Transforms" hud tab ===

        const float k_IndentWidth = 10;
        const float k_TFNameWidth = 136;
        const float k_CheckboxWidth = 35;
        Vector2 m_TransformMenuScrollPosition;

        // Default visualization settings
        Dictionary<TFStream, bool> m_ShowExpanded = new Dictionary<TFStream, bool>();
        public bool ShowTFAxesDefault { get; set; }
        Dictionary<TFStream, bool> m_ShowAxes = new Dictionary<TFStream, bool>();
        public bool ShowTFLinksDefault { get; set; }
        Dictionary<TFStream, bool> m_ShowLinks = new Dictionary<TFStream, bool>();
        public bool ShowTFNamesDefault { get; set; }
        Dictionary<TFStream, bool> m_ShowNames = new Dictionary<TFStream, bool>();

        string IHudTab.Label => "Transforms";

        void IHudTab.OnSelected()
        {
        }

        void IHudTab.OnDeselected()
        {
        }

        void IHudTab.OnGUI(HUDPanel hud)
        {
            m_TransformMenuScrollPosition = GUILayout.BeginScrollView(m_TransformMenuScrollPosition);

            GUI.changed = false;
            GUILayout.BeginHorizontal(GUILayout.Height(20));
            GUILayout.Label("", GUILayout.Width(k_TFNameWidth));
            ShowTFAxesDefault = DrawTFHeaderCheckbox(ShowTFAxesDefault, "Axes", (stream, check) => m_ShowAxes[stream] = check);
            ShowTFLinksDefault = DrawTFHeaderCheckbox(ShowTFLinksDefault, "Link", (stream, check) => m_ShowLinks[stream] = check);
            ShowTFNamesDefault = DrawTFHeaderCheckbox(ShowTFNamesDefault, "Lbl", (stream, check) => m_ShowNames[stream] = check);
            GUILayout.EndHorizontal();
            bool globalChange = GUI.changed;

            // draw the root objects
            int numDrawn = 0;
            foreach (TFStream stream in TFSystem.instance.GetTransforms())
            {
                if (stream.Parent == null)
                {
                    numDrawn++;
                    DrawTFStreamHierarchy(stream, 0, globalChange);
                }
            }

            if (numDrawn == 0)
            {
                GUILayout.Label("(No transform data received yet)");
            }

            GUILayout.EndScrollView();
        }

        void DrawTFStreamHierarchy(TFStream stream, int indent, bool globalChange)
        {
            GUI.changed = false;
            EnsureSettings(stream);

            GUILayout.BeginHorizontal();
            GUILayout.Space(indent * k_IndentWidth);

            /*#if UNITY_EDITOR
                        GUIStyle style = new GUIStyle(UnityEditor.EditorStyles.foldout);
                        style.fixedWidth = k_TFNameWidth;// - indent * k_IndentWidth;
                        style.stretchWidth = false;
                        m_ShowExpanded[stream] = UnityEditor.EditorGUILayout.Foldout(m_ShowExpanded[stream], stream.Name, true, style);// GUILayout.Width(k_TFNameWidth - indent * k_IndentWidth));
            #else*/
            if (GUILayout.Button(stream.Name, GUI.skin.label, GUILayout.Width(k_TFNameWidth - indent * k_IndentWidth)))
                m_ShowExpanded[stream] = !m_ShowExpanded[stream];
            //#endif

            m_ShowAxes[stream] = GUILayout.Toggle(m_ShowAxes[stream], "", GUILayout.Width(k_CheckboxWidth));
            m_ShowLinks[stream] = GUILayout.Toggle(m_ShowLinks[stream], "", GUILayout.Width(k_CheckboxWidth));
            m_ShowNames[stream] = GUILayout.Toggle(m_ShowNames[stream], "", GUILayout.Width(k_CheckboxWidth));
            GUILayout.EndHorizontal();

            if (GUI.changed || globalChange)
            {
                TFSystem.UpdateVisualization(stream);
            }

            if (m_ShowExpanded[stream])
                foreach (TFStream child in stream.Children)
                    DrawTFStreamHierarchy(child, indent + 1, globalChange);
        }

        bool DrawTFHeaderCheckbox(bool wasChecked, string label, Action<TFStream, bool> setter)
        {
            bool result = GUILayout.Toggle(wasChecked, "", GUILayout.Width(k_CheckboxWidth));

            Rect checkbox = GUILayoutUtility.GetLastRect();
            Vector2 textSize = GUI.skin.label.CalcSize(new GUIContent(label));
            Rect labelRect = new Rect(checkbox.xMin - textSize.x, checkbox.yMin, textSize.x, checkbox.height);
            GUI.Label(labelRect, label);

            if (wasChecked != result)
            {
                foreach (TFStream stream in TFSystem.instance.GetTransforms())
                {
                    setter(stream, result);
                }
            }
            return result;
        }
    }
}
