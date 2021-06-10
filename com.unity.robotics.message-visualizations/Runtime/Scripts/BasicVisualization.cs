using System;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicVisualization : IMessageVisualization
    {
        public BasicDrawing basicDrawing;
        public Action guiAction;

        public BasicVisualization(Message newMessage, MessageMetadata newMeta, Action action, BasicDrawing drawing)
        {
            message = newMessage;
            meta = newMeta;
            guiAction = action;
            basicDrawing = drawing;
        }

        public Message message { get; }
        public MessageMetadata meta { get; }

        public bool hasDrawing
        {
            get => basicDrawing != null;
            set => Delete();
        }

        public bool hasAction
        {
            get => guiAction != null;
            set => guiAction = null;
        }

        public void OnGUI()
        {
            if (guiAction != null) guiAction();
        }

        public void Delete()
        {
            guiAction = null;
            basicDrawing.Destroy();
            basicDrawing = null;
        }
    }
}
