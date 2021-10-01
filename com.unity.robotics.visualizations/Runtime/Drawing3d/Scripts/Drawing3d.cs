using RosMessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Robotics.Visualizations
{
    public enum TFTrackingType
    {
        None,
        Exact,
        TrackLatest,
    }

    [System.Serializable]
    public class TFTrackingSettings
    {
        public TFTrackingType type = TFTrackingType.Exact;
        public string tfTopic = "/tf";
    }

    public class Drawing3d : MonoBehaviour
    {
        static GUIStyle s_LabelStyle;

        struct LabelInfo3d
        {
            public Vector3 position;
            public string text;
            public Color color;
            public float worldSpacing;
        }

        Mesh m_Mesh;
        List<Vector3> m_Vertices = new List<Vector3>();
        List<Color32> m_Colors32 = new List<Color32>();
        // When drawing a mesh, it's easier and faster to create a separate object to represent the mesh, than copy its vertices into our drawing
        List<MeshRenderer> m_SupplementalMeshes = new List<MeshRenderer>();
        // For efficiency, when a BasicDrawing is cleared, its supplemental meshes go dormant instead of being deleted. They get reawakened by calling AddMesh.
        Queue<MeshRenderer> m_DormantMeshes = new Queue<MeshRenderer>();
        List<PointCloudDrawing> m_PointClouds = new List<PointCloudDrawing>();
        Queue<PointCloudDrawing> m_DormantPointClouds = new Queue<PointCloudDrawing>();
        List<int> m_Triangles = new List<int>();
        List<LabelInfo3d> m_Labels = new List<LabelInfo3d>();
        bool m_isDirty = false;
        Coroutine m_DestroyAfterDelay;

        public static Drawing3d Create(float duration = -1, Material material = null)
        {
            return Drawing3dManager.CreateDrawing(duration, material);
        }

        public void Init(Drawing3dManager parent, Material material, float duration = -1)
        {
            m_Mesh = new Mesh();

            transform.parent = parent.transform;

            MeshFilter mfilter = gameObject.AddComponent<MeshFilter>();
            mfilter.sharedMesh = m_Mesh;
            MeshRenderer mrenderer = gameObject.AddComponent<MeshRenderer>();
            mrenderer.sharedMaterial = material;

            if (duration >= 0)
            {
                SetDuration(duration);
            }

            if (s_LabelStyle == null)
            {
                s_LabelStyle = new GUIStyle()
                {
                    alignment = TextAnchor.LowerLeft,
                    wordWrap = false,
                };
            }
        }

        public void SetTFTrackingSettings(TFTrackingSettings tfTrackingType, HeaderMsg headerMsg)
        {
            switch (tfTrackingType.type)
            {
                case TFTrackingType.Exact:
                    {
                        TFFrame frame = TFSystem.instance.GetTransform(headerMsg, tfTrackingType.tfTopic);
                        transform.position = frame.translation;
                        transform.rotation = frame.rotation;
                    }
                    break;
                case TFTrackingType.TrackLatest:
                    {
                        transform.parent = TFSystem.instance.GetTransformObject(headerMsg.frame_id, tfTrackingType.tfTopic).transform;
                        transform.localPosition = Vector3.zero;
                        transform.localRotation = Quaternion.identity;
                    }
                    break;
                case TFTrackingType.None:
                    transform.localPosition = Vector3.zero;
                    transform.localRotation = Quaternion.identity;
                    break;
            }
        }

        public void SetDuration(float duration)
        {
            if (m_DestroyAfterDelay != null)
                StopCoroutine(m_DestroyAfterDelay);
            m_DestroyAfterDelay = StartCoroutine(DestroyAfterDelay(duration));
        }

        public void ClearDuration()
        {
            if (m_DestroyAfterDelay != null)
                StopCoroutine(m_DestroyAfterDelay);
        }

        public IEnumerator DestroyAfterDelay(float duration)
        {
            yield return new WaitForSeconds(duration);
            Destroy();
        }

        public void DrawLine(Vector3 from, Vector3 to, Color32 color, float thickness = 0.1f)
        {
            DrawLine(from, to, color, color, thickness);
        }

        public void DrawLine(Vector3 from, Vector3 to, Color32 fromColor, Color32 toColor, float thickness = 0.1f)
        {
            // Could do this by calling DrawCylinder - but hardcoding it is more efficient
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

            for (int Idx = 0; Idx < 4; ++Idx)
                m_Colors32.Add(fromColor);
            for (int Idx = 0; Idx < 4; ++Idx)
                m_Colors32.Add(toColor);

            AddTriangles(start, 0, 2, 4, 4, 2, 6, 1, 5, 2, 2, 5, 6);
            AddTriangles(start, 0, 4, 3, 3, 4, 7, 1, 3, 5, 5, 3, 7);
            AddTriangles(start, 1, 2, 3, 3, 2, 0, 5, 7, 6, 6, 7, 4);
            SetDirty();
        }

        static int[] simpleRingIndices = new int[4] { 3, 2, 1, 0 };
        static int[] pathCornerRingIndices = new int[4] { 0, 3, 2, -2 };

        public void DrawPath(IEnumerable<Vector3> path, Color32 color, float thickness = 0.1f)
        {
            IEnumerator<Vector3> enumerator = path.GetEnumerator();
            if (!enumerator.MoveNext())
                return; // empty path

            Vector3 pointA = enumerator.Current;
            if (!enumerator.MoveNext())
            {
                // only one point in path
                DrawPoint(pointA, color, thickness);
                return;
            }

            Vector3 pointB = enumerator.Current;
            Vector3 lineABDirection = (pointB - pointA).normalized;
            Vector3 oldOutVector = Vector3.Cross(Vector3.up, lineABDirection).normalized * thickness;
            Vector3 oldSideVector = Vector3.Cross(oldOutVector, lineABDirection).normalized * thickness;
            int oldVertexIndex = m_Vertices.Count;
            m_Vertices.Add(pointA + oldOutVector); // 0
            m_Colors32.Add(color);
            m_Vertices.Add(pointA - oldSideVector); // 1
            m_Colors32.Add(color);
            m_Vertices.Add(pointA - oldOutVector); // 2
            m_Colors32.Add(color);
            m_Vertices.Add(pointA + oldSideVector); // 3
            m_Colors32.Add(color);
            AddQuad(oldVertexIndex, 3, 2, 1, 0);

            while (enumerator.MoveNext())
            {
                Vector3 pointC = enumerator.Current;
                // create new vertices around pointB.
                Vector3 lineBCDirection = (pointC - pointB).normalized;
                Vector3 sideVector = Vector3.Cross(lineABDirection, lineBCDirection).normalized * thickness;
                Vector3 abOutVector = Vector3.Cross(lineABDirection, sideVector).normalized * thickness;
                Vector3 bcOutVector = Vector3.Cross(lineBCDirection, sideVector).normalized * thickness;
                Vector3 midOutVector = (abOutVector + bcOutVector).normalized * thickness;
                Vector3 inVector = -midOutVector; // TO DO: make this smarter

                // we draw each corner with 6 vertices: the two perpendiculars, the inside, the outside,
                // and two extra outside points perpendicular to line AB and line BC respectively.
                // We set newVertexIndex in the middle of this set of vertices, because next time around this will become the oldVertexIndex
                // and we want to treat it the same as the initial four vertices we created above
                m_Vertices.Add(pointB + abOutVector); // -2
                m_Colors32.Add(color);
                m_Vertices.Add(pointB + midOutVector); // -1
                m_Colors32.Add(color);
                int newVertexIndex = m_Vertices.Count;
                m_Vertices.Add(pointB + sideVector); // 0
                m_Colors32.Add(color);
                m_Vertices.Add(pointB + bcOutVector); // 1
                m_Colors32.Add(color);
                m_Vertices.Add(pointB - sideVector); // 2
                m_Colors32.Add(color);
                m_Vertices.Add(pointB + inVector); // 3
                m_Colors32.Add(color);
                AddTriangles(newVertexIndex, -2, -1, 0, -2, 2, -1, -1, 2, 1, -1, 1, 0);

                // to avoid having too much twist in the connecting polygons, figure out axis is closest to the old side vector
                float sideDot = Vector3.Dot(oldSideVector, sideVector);
                float outDot = Vector3.Dot(oldSideVector, abOutVector);

                int alignmentIndex;
                if (Mathf.Abs(sideDot) > Mathf.Abs(outDot))
                {
                    alignmentIndex = sideDot > 0 ? 0 : 2;
                }
                else
                {
                    alignmentIndex = outDot > 0 ? 1 : 3;
                }
                ConnectPathRings(oldVertexIndex, newVertexIndex, simpleRingIndices, pathCornerRingIndices, alignmentIndex);

                pointA = pointB;
                pointB = pointC;
                lineABDirection = lineBCDirection;
                oldVertexIndex = newVertexIndex;
                oldSideVector = sideVector;
                oldOutVector = bcOutVector;
            }

            // for the last point, we make another ring
            int finalVertexIndex = m_Vertices.Count;
            m_Vertices.Add(pointB + oldOutVector); // 0
            m_Colors32.Add(color);
            m_Vertices.Add(pointB - oldSideVector); // 1
            m_Colors32.Add(color);
            m_Vertices.Add(pointB - oldOutVector); // 2
            m_Colors32.Add(color);
            m_Vertices.Add(pointB + oldSideVector); // 3
            m_Colors32.Add(color);
            AddQuad(finalVertexIndex, 0, 1, 2, 3);
            ConnectPathRings(oldVertexIndex, finalVertexIndex, simpleRingIndices, simpleRingIndices, 0);
            SetDirty();
        }

        void ConnectPathRings(int fromIndex, int toIndex, int[] fromOffsets, int[] toOffsets, int alignment)
        {
            // connect this line
            for (int Idx = 0; Idx < fromOffsets.Length; ++Idx)
            {
                AddQuad(
                    fromIndex + fromOffsets[((alignment + Idx) % 4)],
                    toIndex + toOffsets[Idx],
                    toIndex + toOffsets[(Idx + 1) % 4],
                    fromIndex + fromOffsets[((alignment + Idx + 1) % 4)]
                );
            }
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

        public void DrawQuad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color32 color, bool doubleSided = false)
        {
            int start = m_Vertices.Count;
            m_Vertices.Add(a);
            m_Colors32.Add(color);
            m_Vertices.Add(b);
            m_Colors32.Add(color);
            m_Vertices.Add(c);
            m_Colors32.Add(color);
            m_Vertices.Add(d);
            m_Colors32.Add(color);
            AddQuad(start, 0, 1, 2, 3);
            if (doubleSided)
                AddQuad(start, 3, 2, 1, 0);
        }

        public void DrawArrow(Vector3 from, Vector3 to, Color32 color, float thickness = 0.1f, float arrowheadScale = 3, float arrowheadGradient = 0.5f)
        {
            float arrowheadRadius = thickness * arrowheadScale;
            float arrowheadLength = arrowheadRadius / arrowheadGradient;
            Vector3 direction = (to - from).normalized;
            Vector3 arrowheadJoint = to - direction * arrowheadLength;
            DrawLine(from, arrowheadJoint, color, thickness);
            DrawCone(arrowheadJoint, to, color, arrowheadRadius);
        }

        public void DrawCuboid(Vector3 center, Vector3 halfSize, Color32 color)
        {
            DrawParallelepiped(center, new Vector3(halfSize.x, 0, 0), new Vector3(0, halfSize.y, 0), new Vector3(0, 0, halfSize.z), color);
        }

        public void DrawCuboid(Vector3 center, Vector3 halfSize, Quaternion rotation, Color32 color)
        {
            DrawParallelepiped(center,
                rotation * new Vector3(halfSize.x, 0, 0),
                rotation * new Vector3(0, halfSize.y, 0),
                rotation * new Vector3(0, 0, halfSize.z),
                color);
        }

        public void DrawParallelepiped(Vector3 center, Vector3 x, Vector3 y, Vector3 z, Color32 color)
        {
            int start = m_Vertices.Count;
            m_Vertices.Add(center - x - y - z); // 0
            m_Vertices.Add(center - x - y + z); // 1
            m_Vertices.Add(center - x + y - z); // 2
            m_Vertices.Add(center - x + y + z); // 3
            m_Vertices.Add(center + x - y - z); // 4
            m_Vertices.Add(center + x - y + z); // 5
            m_Vertices.Add(center + x + y - z); // 6
            m_Vertices.Add(center + x + y + z); // 7
            for (int Idx = 0; Idx < 8; ++Idx)
                m_Colors32.Add(color);
            AddQuad(start, 0, 1, 3, 2); // left face
            AddQuad(start, 4, 6, 7, 5); // right face
            AddQuad(start, 2, 3, 7, 6); // top face
            AddQuad(start, 0, 4, 5, 1); // bottom face
            AddQuad(start, 0, 2, 6, 4); // back face
            AddQuad(start, 1, 5, 7, 3); // front face
            SetDirty();
        }

        public void DrawSphere(Vector3 center, Color32 color, float radius, int numDivisions = 16)
        {
            DrawSpheroid(center, new Vector3(radius, radius, radius), Quaternion.identity, color, numDivisions);
        }

        public void DrawSpheroid(Vector3 center, Vector3 radii, Quaternion rotation, Color32 color, int numDivisions = 16)
        {
            int start = m_Vertices.Count;
            Vector3 upVector = rotation * Vector3.up * radii.y;
            m_Vertices.Add(center + upVector);
            m_Colors32.Add(color);
            m_Vertices.Add(center - upVector);
            m_Colors32.Add(color);

            int numRings = (numDivisions / 2) - 1;
            float yawStep = Mathf.PI * 2 / numDivisions;
            float pitchStep = Mathf.PI / (numRings + 1);
            int lastRingStart = 2 + numRings - 1;

            for (int vertexIdx = 0; vertexIdx < numDivisions; ++vertexIdx)
            {
                float yaw = vertexIdx * yawStep;
                Vector3 sideVector = rotation * new Vector3(Mathf.Sin(yaw) * radii.x, 0, Mathf.Cos(yaw) * radii.z);
                for (int ringIdx = 0; ringIdx < numRings; ++ringIdx)
                {
                    float pitch = Mathf.PI * 0.5f - (ringIdx + 1) * pitchStep;
                    m_Vertices.Add(center + sideVector * Mathf.Cos(pitch) + upVector * Mathf.Sin(pitch));
                    m_Colors32.Add(color);

                    if (ringIdx + 1 < numRings)
                    {
                        AddQuad(start + 2,
                            ((vertexIdx + 1) % numDivisions) * numRings + ringIdx,
                            ((vertexIdx + 1) % numDivisions) * numRings + ringIdx + 1,
                            vertexIdx * numRings + ringIdx + 1,
                            vertexIdx * numRings + ringIdx
                        );
                    }
                }

                AddTriangles(start,
                    0,
                    2 + ((vertexIdx + 1) % numDivisions) * numRings,
                    2 + vertexIdx * numRings,

                    1,
                    lastRingStart + vertexIdx * numRings,
                    lastRingStart + ((vertexIdx + 1) % numDivisions) * numRings
                );
            }
            SetDirty();
        }

        public void DrawCircle(Vector3 center, Vector3 normal, Color32 color, float radius, int numRingVertices = 8)
        {
            DrawCircleOneSided(center, normal, color, radius, numRingVertices);
            DrawCircleOneSided(center, -normal, color, radius, numRingVertices);
        }

        public void DrawCircleOneSided(Vector3 center, Vector3 normal, Color32 color, float radius, int numRingVertices = 8)
        {
            Vector3 forwardVector = ((Mathf.Abs(normal.z) < Mathf.Abs(normal.x)) ? new Vector3(normal.y, -normal.x, 0) : new Vector3(0, -normal.z, normal.y)).normalized * radius;
            Vector3 sideVector = Vector3.Cross(forwardVector, normal).normalized * radius;

            float angleScale = Mathf.PI * 2.0f / numRingVertices;

            int start = m_Vertices.Count;
            m_Vertices.Add(center);
            m_Colors32.Add(color);

            m_Vertices.Add(center + forwardVector);
            m_Colors32.Add(color);
            for (int step = 1; step < numRingVertices; ++step)
            {
                float angle = step * angleScale;
                m_Vertices.Add(center + forwardVector * Mathf.Cos(angle) + sideVector * Mathf.Sin(angle));
                m_Colors32.Add(color);

                m_Triangles.Add(start); // center
                m_Triangles.Add(start + step + 1); // new ring vertex
                m_Triangles.Add(start + step); // previous ring vertex
            }

            m_Triangles.Add(start); // center
            m_Triangles.Add(start + numRingVertices); // last ring vertex
            m_Triangles.Add(start + 1); // first ring vertex
            SetDirty();
        }

        public void DrawCylinder(Vector3 bottom, Vector3 top, Color32 color, float radius, int numRingVertices = 8)
        {
            DrawCylinder(bottom, top, color, color, radius, numRingVertices);
        }

        public void DrawCylinder(Vector3 bottom, Vector3 top, Color32 bottomColor, Color32 topColor, float radius, int numRingVertices = 8)
        {
            int start = m_Vertices.Count;
            Vector3 upVector = top - bottom;
            Vector3 forwardVector = ((Mathf.Abs(upVector.z) < Mathf.Abs(upVector.x)) ? new Vector3(upVector.y, -upVector.x, 0) : new Vector3(0, -upVector.z, upVector.y)).normalized * radius;
            Vector3 sideVector = Vector3.Cross(forwardVector, upVector).normalized * radius;
            float angleScale = Mathf.PI * 2.0f / numRingVertices;

            m_Vertices.Add(bottom);
            m_Colors32.Add(bottomColor);

            m_Vertices.Add(top);
            m_Colors32.Add(topColor);

            m_Vertices.Add(bottom + forwardVector);
            m_Colors32.Add(bottomColor);

            m_Vertices.Add(top + forwardVector);
            m_Colors32.Add(topColor);

            for (int step = 1; step < numRingVertices; ++step)
            {
                float angle = step * angleScale;
                m_Vertices.Add(bottom + forwardVector * Mathf.Cos(angle) + sideVector * Mathf.Sin(angle));
                m_Colors32.Add(bottomColor);

                m_Vertices.Add(top + forwardVector * Mathf.Cos(angle) + sideVector * Mathf.Sin(angle));
                m_Colors32.Add(topColor);

                // bottom circle
                m_Triangles.Add(start); // bottom
                m_Triangles.Add(start + step * 2); // previous bottom ring vertex
                m_Triangles.Add(start + step * 2 + 2); // new bottom ring vertex

                // top circle
                m_Triangles.Add(start + 1); // top
                m_Triangles.Add(start + step * 2 + 3); // new top ring vertex
                m_Triangles.Add(start + step * 2 + 1); // previous top ring vertex

                // cylinder wall
                AddQuad(start + step * 2, 0, 1, 3, 2);
            }

            // bottom circle
            m_Triangles.Add(start); // bottom
            m_Triangles.Add(start + numRingVertices * 2); // last bottom ring vertex
            m_Triangles.Add(start + 2); // first bottom ring vertex

            // top circle
            m_Triangles.Add(start + 1); // top
            m_Triangles.Add(start + 3); // first top ring vertex
            m_Triangles.Add(start + numRingVertices * 2 + 1); // last top ring vertex

            // cylinder wall
            AddQuad(start, 3, 2, numRingVertices * 2, numRingVertices * 2 + 1);
            SetDirty();
        }

        public void DrawCone(Vector3 basePosition, Vector3 tipPosition, Color32 color, float radius, int numRingVertices = 8)
        {
            int start = m_Vertices.Count;
            m_Vertices.Add(basePosition);
            m_Colors32.Add(new Color32((byte)(color.r / 2), (byte)(color.g / 2), (byte)(color.b / 2), 255));
            m_Vertices.Add(tipPosition);
            m_Colors32.Add(color);
            Vector3 heightVector = tipPosition - basePosition;
            Vector3 forwardVector = ((Mathf.Abs(heightVector.z) < Mathf.Abs(heightVector.x)) ? new Vector3(heightVector.y, -heightVector.x, 0) : new Vector3(0, -heightVector.z, heightVector.y)).normalized * radius;
            Vector3 sideVector = Vector3.Cross(forwardVector, heightVector).normalized * radius;
            float angleScale = Mathf.PI * 2.0f / numRingVertices;

            m_Vertices.Add(basePosition + forwardVector);
            m_Colors32.Add(color);

            for (int step = 1; step < numRingVertices; ++step)
            {
                float angle = step * angleScale;
                m_Vertices.Add(basePosition + forwardVector * Mathf.Cos(angle) + sideVector * Mathf.Sin(angle));
                m_Colors32.Add(color);

                m_Triangles.Add(start + 1); // tip
                m_Triangles.Add(start + step + 2); // new ring vertex
                m_Triangles.Add(start + step + 1); // previous ring vertex

                m_Triangles.Add(start); // base
                m_Triangles.Add(start + step + 1); // previous ring vertex
                m_Triangles.Add(start + step + 2); // new ring vertex
            }

            m_Triangles.Add(start + 1); // tip
            m_Triangles.Add(start + 2); // first ring vertex
            m_Triangles.Add(start + numRingVertices + 1); // last ring vertex

            m_Triangles.Add(start); // base
            m_Triangles.Add(start + numRingVertices + 1); // last ring vertex
            m_Triangles.Add(start + 2); // first ring vertex
            SetDirty();
        }

        public void DrawTriangle(Vector3 a, Vector3 b, Vector3 c, Color32 color)
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

        public void DrawTriangle(Vector3 a, Vector3 b, Vector3 c, Color32 colorA, Color32 colorB, Color32 colorC)
        {
            int start = m_Vertices.Count;
            m_Vertices.Add(a);
            m_Colors32.Add(colorA);
            m_Vertices.Add(b);
            m_Colors32.Add(colorB);
            m_Vertices.Add(c);
            m_Colors32.Add(colorC);
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

        public void DrawLines(Vector3[] pointPairs, Color32 color, float thickness)
        {
            for (int Idx = 1; Idx < pointPairs.Length; Idx += 2)
            {
                DrawLine(pointPairs[Idx - 1], pointPairs[Idx], color, thickness);
            }
        }

        public void DrawLines(Vector3[] pointPairs, Color32[] colorPairs, float thickness)
        {
            for (int Idx = 1; Idx < pointPairs.Length; Idx += 2)
            {
                DrawLine(pointPairs[Idx - 1], pointPairs[Idx], colorPairs[Idx - 1], colorPairs[Idx], thickness);
            }
        }

        public void DrawLineStrip(Vector3[] stripPoints, Color32 color, float thickness)
        {
            for (int Idx = 1; Idx < stripPoints.Length; ++Idx)
            {
                DrawLine(stripPoints[Idx - 1], stripPoints[Idx], color, color, thickness);
            }
        }

        public void DrawLineStrip(Vector3[] stripPoints, Color32[] stripColors, float thickness)
        {
            for (int Idx = 1; Idx < stripPoints.Length; ++Idx)
            {
                DrawLine(stripPoints[Idx - 1], stripPoints[Idx], stripColors[Idx - 1], stripColors[Idx], thickness);
            }
        }

        void AddTriangles(int firstIdx, params int[] offsets)
        {
            foreach (int offset in offsets)
            {
                m_Triangles.Add(firstIdx + offset);
            }
        }

        void AddTriangle(int firstIdx, int a, int b, int c)
        {
            m_Triangles.Add(firstIdx + a);
            m_Triangles.Add(firstIdx + b);
            m_Triangles.Add(firstIdx + c);
        }

        // indices in clockwise order
        void AddQuad(int offset, int a, int b, int c, int d)
        {
            AddQuad(offset + a, offset + b, offset + c, offset + d);
        }

        // indices in clockwise order
        void AddQuad(int a, int b, int c, int d)
        {
            m_Triangles.Add(a);
            m_Triangles.Add(b);
            m_Triangles.Add(c);

            m_Triangles.Add(a);
            m_Triangles.Add(c);
            m_Triangles.Add(d);
        }

        public void DrawLabel(string text, Vector3 position, Color color, float worldSpacing = 0)
        {
            m_Labels.Add(new LabelInfo3d { text = text, position = position, color = color, worldSpacing = worldSpacing });
        }

        public void DrawMesh(Mesh source, Transform transform, Color32 color)
        {
            DrawMesh(source, transform.position, transform.rotation, transform.localScale, color);
        }

        public void DrawMesh(Mesh source, Vector3 position, Quaternion rotation, Vector3 scale, Color32 color)
        {
            Material colorMaterial = (color.a < 255) ?
                new Material(Drawing3dManager.instance.UnlitColorAlphaMaterial) :
                new Material(Drawing3dManager.instance.UnlitColorMaterial);
            colorMaterial.color = color;
            DrawMesh(source, position, rotation, scale, colorMaterial);
        }

        public GameObject DrawMesh(Mesh source, Vector3 position, Quaternion rotation, Vector3 scale, Material material)
        {
            GameObject meshObject;
            MeshRenderer mrenderer;
            if (m_DormantMeshes.Count > 0)
            {
                mrenderer = m_DormantMeshes.Dequeue();
                mrenderer.enabled = true;
                meshObject = mrenderer.gameObject;
            }
            else
            {
                meshObject = new GameObject(source.name);
                MeshFilter mfilter = meshObject.AddComponent<MeshFilter>();
                mfilter.sharedMesh = source;
                mrenderer = meshObject.AddComponent<MeshRenderer>();
            }
            meshObject.transform.parent = transform;
            meshObject.transform.position = position;
            meshObject.transform.rotation = rotation;
            meshObject.transform.localScale = scale;
            mrenderer.sharedMaterial = material;
            m_SupplementalMeshes.Add(mrenderer);

            return meshObject;
        }

        public PointCloudDrawing AddPointCloud(int numPoints = 0, Material material = null)
        {
            PointCloudDrawing result;
            if (m_DormantPointClouds.Count > 0)
            {
                result = m_DormantPointClouds.Dequeue();
                result.SetCapacity(numPoints);
                result.SetMaterial(material);
            }
            else
            {
                result = PointCloudDrawing.Create(gameObject, numPoints, material);
            }
            m_PointClouds.Add(result);
            return result;
        }

        public void Clear()
        {
            m_Vertices.Clear();
            m_Colors32.Clear();
            m_Triangles.Clear();
            m_Labels.Clear();
            foreach (MeshRenderer obj in m_SupplementalMeshes)
            {
                obj.enabled = false;
                m_DormantMeshes.Enqueue(obj);
            }
            m_SupplementalMeshes.Clear();
            foreach (PointCloudDrawing pointCloud in m_PointClouds)
            {
                pointCloud.Clear();
                m_DormantPointClouds.Enqueue(pointCloud);
            }
            m_PointClouds.Clear();
            SetDirty();
        }

        public void Destroy()
        {
            GameObject.Destroy(gameObject);
        }

        internal void OnDrawingGUI(Camera cam)
        {
            foreach (LabelInfo3d label in m_Labels)
            {
                Vector3 screenPos = cam.WorldToScreenPoint(transform.TransformPoint(label.position) + cam.transform.right * label.worldSpacing);
                Vector3 guiPos = GUIUtility.ScreenToGUIPoint(screenPos);
                GUI.color = label.color;
                GUIContent labelContent = new GUIContent(label.text);
                Vector2 labelSize = s_LabelStyle.CalcSize(labelContent);
                labelSize.y *= 2;// no idea why we get bad answers for height

                guiPos.y += labelSize.y * 0.35f;
                GUI.Label(new Rect(guiPos.x, Screen.height - guiPos.y, labelSize.x, labelSize.y), label.text);
            }
        }

#if UNITY_EDITOR
        void OnDrawGizmos()
        {
            if (m_Labels.Count == 0)
                return;
            Camera cam = UnityEditor.SceneView.currentDrawingSceneView.camera;
            GUIStyle style = new GUIStyle();
            foreach (LabelInfo3d label in m_Labels)
            {
                style.normal.textColor = label.color;
                Vector3 worldPos = transform.TransformPoint(label.position + cam.transform.right * label.worldSpacing);
                UnityEditor.Handles.Label(worldPos, label.text, style);
            }
        }

#endif

        public void Refresh()
        {
            m_isDirty = false;
            m_Mesh.Clear();
            m_Mesh.indexFormat = m_Vertices.Count < 65536 ? UnityEngine.Rendering.IndexFormat.UInt16 : UnityEngine.Rendering.IndexFormat.UInt32;
            m_Mesh.vertices = m_Vertices.ToArray();
            m_Mesh.colors32 = m_Colors32.ToArray();
            m_Mesh.triangles = m_Triangles.ToArray();
        }

        void SetDirty()
        {
            if (!m_isDirty)
            {
                m_isDirty = true;
                Drawing3dManager.instance.AddDirty(this);
            }
        }
    }
}
