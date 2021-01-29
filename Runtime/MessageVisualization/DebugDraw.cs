using System.Collections.Generic;
using UnityEngine;

public class DebugDraw : MonoBehaviour
{
    static DebugDraw _instance;
    static GUIStyle labelStyle;
    public static DebugDraw instance
    {
        get
        {
            if (_instance == null)
            {
                GameObject newDebugDrawObj = new GameObject("DebugDraw");
                _instance = newDebugDrawObj.AddComponent<DebugDraw>();
                _instance.material = new Material(Shader.Find("Unlit/VertexColorDebug"));
            }
            return _instance;
        }
    }

    Camera cam;
    List<Drawing> drawings = new List<Drawing>();
    List<Drawing> dirty = new List<Drawing>();
    public Material material;
    const int DrawLayer = 4;

    private void Awake()
    {
        _instance = this;
        labelStyle = new GUIStyle()
        {
            alignment = TextAnchor.LowerLeft,
            wordWrap = false,
        };
        cam = Camera.main;
    }

    public Drawing CreateDrawing(float duration = -1, Material material = null)
    {
        Drawing newDrawing = new Drawing(this, material?? this.material, duration >= 0? Time.time + duration: -1);
        drawings.Add(newDrawing);
        return newDrawing;
    }

    private void LateUpdate()
    {
        foreach (Drawing drawing in dirty)
            drawing.Refresh();
        dirty.Clear();

        foreach (Drawing drawing in drawings)
            drawing.Render();
    }


    public void OnGUI()
    {
        Color oldColor = GUI.color;

        foreach (Drawing drawing in drawings)
            drawing.OnGUI(cam);

        GUI.color = oldColor;
    }

    public class Drawing
    {
        Mesh mesh = new Mesh();
        List<Vector3> vertices = new List<Vector3>();
        List<Color32> colors32 = new List<Color32>();
        List<int> triangles = new List<int>();
        float destroyAfterTimestamp;
        DebugDraw parent;
        Material material;
        bool isDirty = false;

        public Drawing(DebugDraw parent, Material material, float destroyAfterTimestamp)
        {
            this.parent = parent;
            this.material = material;
            this.destroyAfterTimestamp = destroyAfterTimestamp;
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
            int start = vertices.Count;
            vertices.Add(from + sideVector); //0
            vertices.Add(from - sideVector);
            vertices.Add(from + upVector); //2
            vertices.Add(from - upVector);
            vertices.Add(to + sideVector);//4
            vertices.Add(to - sideVector);
            vertices.Add(to + upVector);// 6
            vertices.Add(to - upVector);

            for (int Idx = 0; Idx < 8; ++Idx)
                colors32.Add(color);

            AddTriangles(start, 0, 2, 4, 4, 2, 6, 1, 5, 2, 2, 5, 6);
            AddTriangles(start, 0, 4, 3, 3, 4, 7, 1, 3, 5, 5, 3, 7);
            AddTriangles(start, 1, 2, 3, 3, 2, 0, 5, 7, 6, 6, 7, 4);
            SetDirty();
        }

        public void DrawPoint(Vector3 point, Color32 color, float radius = 0.1f)
        {
            // draw a point as an octahedron
            int start = vertices.Count;
            vertices.Add(point + new Vector3(radius, 0, 0));
            vertices.Add(point + new Vector3(-radius, 0, 0));
            vertices.Add(point + new Vector3(0, radius, 0));
            vertices.Add(point + new Vector3(0, -radius, 0));
            vertices.Add(point + new Vector3(0, 0, radius));
            vertices.Add(point + new Vector3(0, 0, -radius));

            for (int Idx = 0; Idx < 6; ++Idx)
                colors32.Add(color);

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
            int start = vertices.Count;
            vertices.Add(a);
            vertices.Add(b);
            vertices.Add(c);
            for (int Idx = 0; Idx < 3; ++Idx)
                colors32.Add(color);
            AddTriangles(start, 0,1,2);
            SetDirty();
        }

        public void DrawTriangleFan(Color32 color, Vector3 center, Vector3 first, params Vector3[] fanPoints)
        {
            int centerIdx = vertices.Count;
            vertices.Add(center);
            colors32.Add(color);
            int currentIdx = vertices.Count;
            vertices.Add(first);
            colors32.Add(color);
            foreach (Vector3 point in fanPoints)
            {
                vertices.Add(point);
                colors32.Add(color);
                triangles.Add(centerIdx);
                triangles.Add(currentIdx);
                triangles.Add(currentIdx+1);
                currentIdx++;
            }
            SetDirty();
        }

        public void DrawTriangleStrip(Color32 color, Vector3 first, Vector3 second, params Vector3[] otherPoints)
        {
            int currentIdx = vertices.Count;
            vertices.Add(first);
            colors32.Add(color);
            vertices.Add(second);
            colors32.Add(color);
            foreach(Vector3 point in otherPoints)
            {
                vertices.Add(point);
                colors32.Add(color);
                AddTriangles(currentIdx, 0, 1, 2);
                currentIdx++;
            }
            SetDirty();
        }

        public void DrawLines(Color32 color, float thickness, params Vector3[] stripPoints)
        {
            for (int Idx = 1; Idx < stripPoints.Length; Idx+=2)
            {
                DrawLine(stripPoints[Idx-1], stripPoints[Idx], color, thickness);
            }
        }

        public void DrawLineStrip(Color32 color, float thickness, params Vector3[] stripPoints)
        {
            for(int Idx = 1; Idx < stripPoints.Length; ++Idx)
            {
                DrawLine(stripPoints[Idx-1], stripPoints[Idx], color, thickness);
            }
        }

        void AddTriangles(int firstIdx, params int[] offsets)
        {
            foreach (int offset in offsets)
            {
                triangles.Add(firstIdx + offset);
            }
        }

        struct LabelInfo3D
        {
            public Vector3 position;
            public string text;
            public Color color;
            public float worldSpacing;
        }

        List<LabelInfo3D> labels = new List<LabelInfo3D>();

        public void DrawLabel(string text, Vector3 position, Color color, float worldSpacing = 0)
        {
            labels.Add(new LabelInfo3D { text = text, position = position, color = color, worldSpacing = worldSpacing });
        }

        public void Clear()
        {
            vertices.Clear();
            colors32.Clear();
            triangles.Clear();
            labels.Clear();
            SetDirty();
        }

        public void Destroy()
        {
            DebugDraw.instance.drawings.Remove(this);
        }

        internal void OnGUI(Camera cam)
        {
            foreach (LabelInfo3D label in labels)
            {
                Vector3 screenPos = cam.WorldToScreenPoint(label.position + cam.transform.right * label.worldSpacing);
                Vector3 guiPos = GUIUtility.ScreenToGUIPoint(screenPos);
                GUI.color = label.color;
                GUIContent labelContent = new GUIContent(label.text);
                Vector2 labelSize = DebugDraw.labelStyle.CalcSize(labelContent);
                labelSize.y *= 2;// no idea why we get bad answers for height

                guiPos.y += labelSize.y * 0.35f;
                GUI.Label(new Rect(guiPos.x, Screen.height - guiPos.y, labelSize.x, labelSize.y), label.text);
            }
        }

        public void Refresh()
        {
            if (destroyAfterTimestamp > 0 && Time.time > destroyAfterTimestamp)
            {
                Destroy();
            }
            else
            {
                mesh.Clear();
                mesh.vertices = vertices.ToArray();
                mesh.colors32 = colors32.ToArray();
                mesh.triangles = triangles.ToArray();
                isDirty = false;
            }
        }

        void SetDirty()
        {
            if (!isDirty)
            {
                isDirty = true;
                parent.dirty.Add(this);
            }
        }

        internal void Render()
        {
            if (destroyAfterTimestamp > 0 && Time.time > destroyAfterTimestamp)
            {
                // can't delete it here, but we can set it dirty. Refresh will destroy it.
                SetDirty();
            }
            else
            {
                Graphics.DrawMesh(mesh, Matrix4x4.identity, material, DebugDraw.DrawLayer);
            }
        }
    }
}
