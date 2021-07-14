using UnityEngine;

namespace Unity.Robotics.ROSTCPConnector.TransformManagement
{
    // Represents a transform - position and rotation.
    //(Like the Unity Transform class, but without the GameObject baggage that comes with it.)
    public readonly struct TransformFrame
    {
        public Vector3 Translation { get; }
        public Quaternion Rotation { get; }

        public static TransformFrame Identity = new TransformFrame(Vector3.zero, Quaternion.identity);

        public TransformFrame(Vector3 translation, Quaternion rotation)
        {
            this.Translation = translation;
            this.Rotation = rotation;
        }

        public Vector3 TransformPoint(Vector3 point)
        {
            return Translation + Rotation * point;
        }

        public Vector3 InverseTransformPoint(Vector3 point)
        {
            return Quaternion.Inverse(Rotation) * (point - Translation);
        }

        public TransformFrame Compose(TransformFrame child)
        {
            return new TransformFrame(TransformPoint(child.Translation), Rotation * child.Rotation);
        }

        public static TransformFrame Lerp(TransformFrame a, TransformFrame b, float lerp)
        {
            return new TransformFrame(
                translation: Vector3.Lerp(a.Translation, b.Translation, lerp),
                rotation: Quaternion.Lerp(a.Rotation, b.Rotation, lerp));
        }

        public override string ToString()
        {
            return $"{{pos: ({Translation}) | rot: ({Rotation.eulerAngles})}}";
        }
    }
}
