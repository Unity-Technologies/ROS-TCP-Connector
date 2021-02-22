using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public abstract class VisualizerConfigWithDrawing<Msg> : VisualizerConfig<Msg> where Msg : Message
    {
        public string label;
        public Color color;

        [HideInInspector][SerializeField]
        string topicCopy;

        public void OnValidate()
        {
            if(topic != topicCopy)
            {
                if (color == Color.clear || color == Color.black || color == MessageVisualizations.PickColorForTopic(topicCopy))
                    color = MessageVisualizations.PickColorForTopic(topic);
                if (label == topicCopy || label == "")
                    label = topic;
            }

            if (color.a == 0)
                color.a = 1;

            topicCopy = topic;
        }

        public override object CreateDrawing(Msg msg, MessageMetadata meta)
        {
            Color colorToUse = this.color;
            string labelToUse = this.label;
            
            if (colorToUse == Color.clear || colorToUse == Color.black)
                colorToUse = MessageVisualizations.PickColorForTopic(meta.topic);
            
            if (labelToUse == "" || labelToUse == null)
                labelToUse = meta.topic;

            DebugDraw.Drawing drawing = DebugDraw.CreateDrawing();
            Draw(msg, meta, colorToUse, labelToUse, drawing);
            return drawing;
        }

        public virtual void Draw(Msg msg, MessageMetadata meta, Color color, string label, DebugDraw.Drawing drawing)
        {
        }

        public override void DeleteDrawing(object drawing)
        {
            ((DebugDraw.Drawing)drawing).Destroy();
        }
    }
}