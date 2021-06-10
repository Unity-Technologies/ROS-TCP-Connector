using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace Unity.Robotics.MessageVisualizers
{
    public class BasicVisualization : IMessageVisualization
    {
        public Message message { get; private set; }
        public MessageMetadata meta { get; private set; }
        public Action guiAction;
        public BasicDrawing basicDrawing;
        public bool hasDrawing 
        {
            get { return basicDrawing != null; }
            set { Delete(); }
        }
        public bool hasAction 
        {
            get { return guiAction != null; }
            set { guiAction = null; }
        }

        public BasicVisualization(Message newMessage, MessageMetadata newMeta, Action action, BasicDrawing drawing)
        {
            message = newMessage;
            meta = newMeta;
            guiAction = action;
            basicDrawing = drawing;
        }

        public void OnGUI()
        {
            if (guiAction != null)
            {
                guiAction();
            }
        }

        public void Delete()
        {
            guiAction = null;
            basicDrawing.Destroy();
            basicDrawing = null;
        }
    }
}
