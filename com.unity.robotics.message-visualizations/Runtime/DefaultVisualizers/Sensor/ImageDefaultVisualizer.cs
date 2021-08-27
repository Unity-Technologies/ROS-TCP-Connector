using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.MessageVisualizers;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class ImageDefaultVisualizer : TextureVisualizer<ImageMsg>
{
    [SerializeField]
    bool m_Debayer = true;

    public override Texture2D CreateTexture(ImageMsg message)
    {
        return message.data.Length > 0 ? message.ToTexture2D(m_Debayer) : null;
    }

    protected override IVisual CreateVisual()
    {
        return new ImageVisual(this);
    }

    public class ImageVisual : ITextureVisual
    {
        // cache the original image height, width and encoding here so that we if we have to debayer the image (which resizes it) we still show the real values
        int m_Width;
        int m_Height;
        string m_Encoding;
        bool m_Debayer;

        ImageDefaultVisualizer m_Factory;

        Action m_GUIAction;
        // after anyone asks for it, we cache the properly processed texture here
        Texture2D m_Texture2D;
        // if nobody asks for the proper texture, we generate CheapTexture2D: this is just
        // the raw bytes from the message, dumped into a texture.
        // cheap to make but will look wrong without a special shader
        Texture2D m_CheapTexture2D;
        Material m_CheapTextureMaterial;
        List<Action<Texture2D>> m_OnChangeCallbacks = new List<Action<Texture2D>>();

        public void ListenForTextureChange(Action<Texture2D> callback)
        {
            m_OnChangeCallbacks.Add(callback);
        }

        public ImageVisual(ImageDefaultVisualizer factory)
        {
            m_Factory = factory;
            m_Debayer = factory.m_Debayer;
            m_CheapTextureMaterial = new Material(Shader.Find("Unlit/ImageMsg"));
        }

        public virtual void AddMessage(Message message, MessageMetadata meta)
        {
            if (!MessageVisualizationUtils.AssertMessageType<ImageMsg>(message, meta))
                return;

            this.message = (ImageMsg)message;
            this.meta = meta;
            m_Texture2D = null;
            m_CheapTexture2D = null;
            m_GUIAction = null;
            m_Width = (int)this.message.width;
            m_Height = (int)this.message.height;
            m_Encoding = this.message.encoding;
            //m_CheapTextureMaterial.SetFloat("_gray", this.message.GetNumChannels() == 1 ? 1.0f : 0.0f);
            m_CheapTextureMaterial.SetFloat("_convertBGR", this.message.EncodingRequiresBGRConversion() ? 1.0f : 0.0f);

            // if anyone wants to know about the texture, make sure it's updated for them
            if (m_OnChangeCallbacks.Count > 0)
                GetTexture();
        }

        public ImageMsg message { get; private set; }

        public MessageMetadata meta { get; private set; }

        public bool hasDrawing => false;
        public bool hasAction => m_GUIAction != null;

        public void OnGUI()
        {
            message.header.GUI();
            GUILayout.Label($"{m_Height}x{m_Width}, encoding: {m_Encoding}");
            if (message.data.Length > 0)
            {
                GUILayout.BeginHorizontal();
                if (m_Encoding.StartsWith("bayer"))
                    m_Debayer = GUILayout.Toggle(m_Debayer, "Debayer");
                GUILayout.EndHorizontal();

                if (m_Texture2D != null || m_Debayer)
                {
                    // if we already generated the "real" texture, just use that
                    GetTexture().GUITexture();
                }
                else
                {
                    if (m_CheapTexture2D == null)
                    {
                        m_CheapTexture2D = message.ToTexture2D(m_Debayer, convertBGR: false, flipY: false);
                    }
                    m_CheapTexture2D.GUITexture(m_CheapTextureMaterial);
                }
            }
        }

        public Texture2D GetTexture()
        {
            if (m_Texture2D == null)
            {
                m_Texture2D = (message != null && message.data.Length > 0) ? message.ToTexture2D(m_Debayer) : null;

                foreach (Action<Texture2D> callback in m_OnChangeCallbacks)
                    callback(m_Texture2D);
            }
            return m_Texture2D;
        }

        public void DeleteDrawing() { }
        public void CreateDrawing() { }
    }
}
