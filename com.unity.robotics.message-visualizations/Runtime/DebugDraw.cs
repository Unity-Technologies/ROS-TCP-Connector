using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class DebugDraw : MonoBehaviour
    {
        static DebugDraw s_Instance;
        static GUIStyle s_LabelStyle;
        public static DebugDraw instance
        {
            get
            {
                if (s_Instance == null)
                {
                    GameObject newDebugDrawObj = new GameObject("DebugDraw");
                    s_Instance = newDebugDrawObj.AddComponent<DebugDraw>();
                    s_Instance.Material = new Material(Shader.Find("Unlit/VertexColor"));
                    s_Instance.UnlitColorMaterial = new Material(Shader.Find("Unlit/Color"));
                    s_Instance.UnlitColorAlphaMaterial = new Material(Shader.Find("Unlit/ColorAlpha"));
                }
                return s_Instance;
            }
        }

        Camera m_Camera;
        List<Drawing> m_Drawings = new List<Drawing>();
        List<Drawing> m_Dirty = new List<Drawing>();
        public Material Material { get; private set; }
        public Material UnlitColorMaterial { get; private set; }
        public Material UnlitColorAlphaMaterial { get; private set; }

        void Awake()
        {
            s_Instance = this;
            s_LabelStyle = new GUIStyle()
            {
                alignment = TextAnchor.LowerLeft,
                wordWrap = false,
            };
            m_Camera = Camera.main;
        }

        public static Drawing CreateDrawing(float duration = -1, Material material = null)
        {
            GameObject newDrawingObj = new GameObject("Drawing");
            Drawing newDrawing = newDrawingObj.AddComponent<Drawing>();
            newDrawing.Init(instance, material ?? instance.Material, duration);
            instance.m_Drawings.Add(newDrawing);
            return newDrawing;
        }

        void LateUpdate()
        {
            foreach (Drawing drawing in m_Dirty)
                drawing.Refresh();
            m_Dirty.Clear();
        }

        public void OnGUI()
        {
            Color oldColor = GUI.color;

            foreach (Drawing drawing in m_Drawings)
                drawing.OnDrawingGUI(m_Camera);

            GUI.color = oldColor;
        }

        public class Drawing: MonoBehaviour
        {
            struct LabelInfo3D
            {
                public Vector3 position;
                public string text;
                public Color color;
                public float worldSpacing;
            }

            Mesh m_Mesh;
            List<Vector3> m_Vertices = new List<Vector3>();
            List<Color32> m_Colors32 = new List<Color32>();
            List<GameObject> m_Supplemental = new List<GameObject>();
            List<int> m_Triangles = new List<int>();
            List<LabelInfo3D> m_Labels = new List<LabelInfo3D>();
            bool isDirty = false;

            public void Init(DebugDraw parent, Material material, float duration = -1)
            {
                m_Mesh = new Mesh();

                this.transform.parent = parent.transform;

                MeshFilter mfilter = gameObject.AddComponent<MeshFilter>();
                mfilter.sharedMesh = m_Mesh;
                MeshRenderer mrenderer = gameObject.AddComponent<MeshRenderer>();
                mrenderer.sharedMaterial = material;

                if(duration >= 0)
                {
                    StartCoroutine(DestroyAfterDelay(duration));
                }
            }

            public IEnumerator DestroyAfterDelay(float duration)
            {
                yield return new WaitForSeconds(duration);
                Destroy();
            }

            public void DrawLine(Vector3 from, Vector3 to, Color32 color, float thickness = 0.1f)
            {
                Vector3 forwardVector = (to - from).normalized;
                Vector3 sideVector;
                if (Vector3.Dot(forwardVector, Vector3.up) > 0.9f) // just want any vector perpendicular to forwardVector
                    sideVector = Vector3.Cross(forwardVector, Vector3.forward).normalized * thickness;
                else
                    sideVector = Vector3.Cross(forwardVector, Vector3.up).normalized * thickness;
                Vector3 upVector = Vector3.Cross(forwardVector, sideVector).normalized * thickness;
                int start = m_Vertices.Count;
                m_Vertices.Add(from + sideVector); //0
                m_Vertices.Add(from - sideVector);
                m_Vertices.Add(from + upVector); //2
                m_Vertices.Add(from - upVector);
                m_Vertices.Add(to + sideVector);//4
                m_Vertices.Add(to - sideVector);
                m_Vertices.Add(to + upVector);// 6
                m_Vertices.Add(to - upVector);

                for (int Idx = 0; Idx < 8; ++Idx)
                    m_Colors32.Add(color);

                AddTriangles(start, 0, 2, 4, 4, 2, 6, 1, 5, 2, 2, 5, 6);
                AddTriangles(start, 0, 4, 3, 3, 4, 7, 1, 3, 5, 5, 3, 7);
                AddTriangles(start, 1, 2, 3, 3, 2, 0, 5, 7, 6, 6, 7, 4);
                SetDirty();
            }

            public void DrawPoint(Vector3 point, Color32 color, float radius = 0.1f)
            {
                // draw a point as an octahedron
                int start = m_Vertices.Count;
                m_Vertices.Add(point + new Vector3(radius, 0, 0));
                m_Vertices.Add(point + new Vector3(-radius, 0, 0));
                m_Vertices.Add(point + new Vector3(0, radius, 0));
                m_Vertices.Add(point + new Vector3(0, -radius, 0));
                m_Vertices.Add(point + new Vector3(0, 0, radius));
                m_Vertices.Add(point + new Vector3(0, 0, -radius));

                for (int Idx = 0; Idx < 6; ++Idx)
                    m_Colors32.Add(color);

                AddTriangles(start, 0, 2, 4, 0, 5, 2, 0, 4, 3, 0, 3, 5);
                AddTriangles(start, 1, 4, 2, 1, 2, 5, 1, 3, 4, 1, 5, 3);
                SetDirty();
            }

            public void DrawPoint(Vector3 point, string label, Color32 color, float radius = 0.1f)
            {
                DrawPoint(point, color, radius);
                DrawLabel(label, point, color, radius * 1.5f);
            }

            public void DrawTriangle(Color32 color, Vector3 a, Vector3 b, Vector3 c)
            {
                int start = m_Vertices.Count;
                m_Vertices.Add(a);
                m_Vertices.Add(b);
                m_Vertices.Add(c);
                for (int Idx = 0; Idx < 3; ++Idx)
                    m_Colors32.Add(color);
                AddTriangles(start, 0, 1, 2);
                SetDirty();
            }

            public void DrawTriangleFan(Color32 color, Vector3 center, Vector3 first, params Vector3[] fanPoints)
            {
                int centerIdx = m_Vertices.Count;
                m_Vertices.Add(center);
                m_Colors32.Add(color);
                int currentIdx = m_Vertices.Count;
                m_Vertices.Add(first);
                m_Colors32.Add(color);
                foreach (Vector3 point in fanPoints)
                {
                    m_Vertices.Add(point);
                    m_Colors32.Add(color);
                    m_Triangles.Add(centerIdx);
                    m_Triangles.Add(currentIdx);
                    m_Triangles.Add(currentIdx + 1);
                    currentIdx++;
                }
                SetDirty();
            }

            public void DrawTriangleStrip(Color32 color, Vector3 first, Vector3 second, params Vector3[] otherPoints)
            {
                int currentIdx = m_Vertices.Count;
                m_Vertices.Add(first);
                m_Colors32.Add(color);
                m_Vertices.Add(second);
                m_Colors32.Add(color);
                foreach (Vector3 point in otherPoints)
                {
                    m_Vertices.Add(point);
                    m_Colors32.Add(color);
                    AddTriangles(currentIdx, 0, 1, 2);
                    currentIdx++;
                }
                SetDirty();
            }

            public void DrawLines(Color32 color, float thickness, params Vector3[] pointPairs)
            {
                for (int Idx = 1; Idx < pointPairs.Length; Idx += 2)
                {
                    DrawLine(pointPairs[Idx - 1], pointPairs[Idx], color, thickness);
                }
            }

            public void DrawLineStrip(Color32 color, float thickness, params Vector3[] stripPoints)
            {
                for (int Idx = 1; Idx < stripPoints.Length; ++Idx)
                {
                    DrawLine(stripPoints[Idx - 1], stripPoints[Idx], color, thickness);
                }
            }

            void AddTriangles(int firstIdx, params int[] offsets)
            {
                foreach (int offset in offsets)
                {
                    m_Triangles.Add(firstIdx + offset);
                }
            }

            public void DrawLabel(string text, Vector3 position, Color color, float worldSpacing = 0)
            {
                m_Labels.Add(new LabelInfo3D { text = text, position = position, color = color, worldSpacing = worldSpacing });
            }

            public void DrawMesh(Mesh source, Transform transform, Color32 color)
            {
                DrawMesh(source, transform.position, transform.rotation, transform.localScale, color);
            }

            public void DrawMesh(Mesh source, Vector3 position, Quaternion rotation, Vector3 scale, Color32 color)
            {
                GameObject meshObject = new GameObject(source.name);
                m_Supplemental.Add(meshObject);
                meshObject.transform.parent = transform;
                meshObject.transform.position = position;
                meshObject.transform.rotation = rotation;
                meshObject.transform.localScale = scale;
                MeshFilter mfilter = meshObject.AddComponent<MeshFilter>();
                mfilter.sharedMesh = source;
                MeshRenderer mrenderer = meshObject.AddComponent<MeshRenderer>();
                mrenderer.material = (color.a < 255) ? DebugDraw.instance.UnlitColorAlphaMaterial : DebugDraw.instance.UnlitColorMaterial;
                mrenderer.material.color = color;
            }

            public void Clear()
            {
                m_Vertices.Clear();
                m_Colors32.Clear();
                m_Triangles.Clear();
                m_Labels.Clear();
                foreach (GameObject obj in m_Supplemental)
                    GameObject.Destroy(obj);
                m_Supplemental.Clear();
                SetDirty();
            }

            public void Destroy()
            {
                DebugDraw.instance.m_Drawings.Remove(this);
                GameObject.Destroy(gameObject);
            }

            internal void OnDrawingGUI(Camera cam)
            {
                foreach (LabelInfo3D label in m_Labels)
                {
                    Vector3 screenPos = cam.WorldToScreenPoint(label.position + cam.transform.right * label.worldSpacing);
                    Vector3 guiPos = GUIUtility.ScreenToGUIPoint(screenPos);
                    GUI.color = label.color;
                    GUIContent labelContent = new GUIContent(label.text);
                    Vector2 labelSize = DebugDraw.s_LabelStyle.CalcSize(labelContent);
                    labelSize.y *= 2;// no idea why we get bad answers for height

                    guiPos.y += labelSize.y * 0.35f;
                    GUI.Label(new Rect(guiPos.x, Screen.height - guiPos.y, labelSize.x, labelSize.y), label.text);
                }
            }

            public void Refresh()
            {
                m_Mesh.Clear();
                m_Mesh.vertices = m_Vertices.ToArray();
                m_Mesh.colors32 = m_Colors32.ToArray();
                m_Mesh.triangles = m_Triangles.ToArray();
                isDirty = false;
            }

            void SetDirty()
            {
                if (!isDirty)
                {
                    isDirty = true;
                    instance.m_Dirty.Add(this);
                }
            }
        }
    }
}