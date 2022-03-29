using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using Unity.Robotics.Visualizations;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class ImageDefaultVisualizer : BaseVisualFactory<ImageMsg>
{
    [SerializeField]
    [Tooltip("Pixels in a Unity texture are upside down relative to the ones in a ROS image")]
    bool m_FlipVertical = true;
    [SerializeField]
    [Tooltip("Convert bayer encoded images into RGB color?")]
    bool m_Debayer = true;
    [SerializeField]
    [Tooltip("If set, single channel images are mapped onto this gradient. (If not set, they are displayed as red.)")]
    Texture2D m_SingleChannelToGradient;
    [SerializeField]
    [Tooltip("Scale image pixel values by this amount. (For use when visualizing depth and HDR images.)")]
    float m_BrightnessMultiplier = 1.0f;

    public override bool CanShowDrawing => false;

    protected override IVisual CreateVisual(string topic)
    {
        return new ImageVisual(topic, this);
    }

    public class ImageVisual : ITextureVisual
    {
        string m_Topic;

        // cache the original image height, width and encoding here so that we if we have to debayer the image (which resizes it) we still show the real values
        int m_Width;
        int m_Height;
        string m_Encoding;
        bool m_MessageIsBayerEncoded;
        bool m_MessageIsSingleChannel;
        bool m_Debayer;

        ImageDefaultVisualizer m_Factory;

        // after anyone asks for it, we cache the properly processed texture here
        Texture2D m_Texture2D;
        bool m_Texture2DIsDirty;
        string m_Texture2DEncoding;
        // if nobody asks for the proper texture, we generate CheapTexture2D: this is just
        // the raw bytes from the message, dumped into a texture.
        // cheap to make but will look wrong without a special shader
        Texture2D m_CheapTexture2D;
        bool m_CheapTexture2DIsDirty;
        string m_CheapTexture2DEncoding;
        Material m_CheapTextureMaterial;
        Material m_ImageGradientMaterial;

        List<Action<Texture2D>> m_OnChangeCallbacks = new List<Action<Texture2D>>();

        public void ListenForTextureChange(Action<Texture2D> callback)
        {
            m_OnChangeCallbacks.Add(callback);
        }

        public ImageVisual(string topic, ImageDefaultVisualizer factory)
        {
            m_Topic = topic;
            m_Factory = factory;
            m_Debayer = m_Factory.m_Debayer;
            m_CheapTextureMaterial = new Material(Shader.Find("Unlit/ImageMsg"));
            m_ImageGradientMaterial = new Material(Shader.Find("Unlit/ImageGradient"));

            ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(m_Topic, AddMessage);
        }

        public virtual void AddMessage(Message message)
        {
            if (!VisualizationUtils.AssertMessageType<ImageMsg>(message, m_Topic))
                return;

            this.message = (ImageMsg)message;
            m_Texture2DIsDirty = true;
            m_CheapTexture2DIsDirty = true;
            m_Width = (int)this.message.width;
            m_Height = (int)this.message.height;
            m_Encoding = this.message.encoding;
            m_MessageIsBayerEncoded = m_Encoding.StartsWith("bayer");
            m_MessageIsSingleChannel = this.message.GetNumChannels() == 1;
            //m_CheapTextureMaterial.SetFloat("_gray", this.message.GetNumChannels() == 1 ? 1.0f : 0.0f);

            // if anyone wants to know about the texture, notify them
            if (m_OnChangeCallbacks.Count > 0)
            {
                GetTexture(); // force m_Texture2D to update
                foreach (Action<Texture2D> callback in m_OnChangeCallbacks)
                    callback(m_Texture2D);
            }
        }

        public ImageMsg message { get; private set; }

        static int ConvertBGRPropertyID = Shader.PropertyToID("_convertBGR");
        static int GradientPropertyId = Shader.PropertyToID("_Gradient");
        static int BrightnessMultiplierPropertyId = Shader.PropertyToID("_BrightnessMultiplier");
        static int FlipYPropertyId = Shader.PropertyToID("_flipY");

        public void OnGUI()
        {
            if (message == null)
            {
                GUILayout.Label("Waiting for message...");
                return;
            }

            message.header.GUI();
            if (message.data.Length <= 0)
            {
                GUILayout.Label($"{m_Height}x{m_Width}, encoding: {m_Encoding}");
                return;
            }

            GUILayout.BeginHorizontal();
            GUILayout.Label($"{m_Height}x{m_Width}, encoding: {m_Encoding}");
            if (m_MessageIsBayerEncoded)
                m_Debayer = GUILayout.Toggle(m_Debayer, "Debayer");
            GUILayout.EndHorizontal();

            if (m_Texture2D != null || (m_Debayer && m_MessageIsBayerEncoded))
            {
                // if we already generated the "real" texture, just use that
                GetTexture().GUITexture();
            }
            else
            {
                TextureRefresh(ref m_CheapTexture2D, ref m_CheapTexture2DEncoding, ref m_CheapTexture2DIsDirty, doConversion: false);

                if (m_Factory.m_SingleChannelToGradient != null && m_MessageIsSingleChannel)
                {
                    m_ImageGradientMaterial.SetTexture(GradientPropertyId, m_Factory.m_SingleChannelToGradient);
                    m_ImageGradientMaterial.SetFloat(BrightnessMultiplierPropertyId, m_Factory.m_BrightnessMultiplier);
                    m_ImageGradientMaterial.SetFloat(FlipYPropertyId, m_Factory.m_FlipVertical ? 1.0f : 0.0f);
                    m_CheapTexture2D.GUITexture(m_ImageGradientMaterial);
                }
                else
                {
                    m_CheapTextureMaterial.SetFloat(ConvertBGRPropertyID, message.EncodingRequiresBGRConversion() ? 1.0f : 0.0f);
                    m_CheapTextureMaterial.SetFloat(BrightnessMultiplierPropertyId, m_Factory.m_BrightnessMultiplier);
                    m_CheapTextureMaterial.SetFloat(FlipYPropertyId, m_Factory.m_FlipVertical ? 1.0f : 0.0f);
                    m_CheapTexture2D.GUITexture(m_CheapTextureMaterial);
                }
            }
        }

        public Texture2D GetTexture()
        {
            TextureRefresh(ref m_Texture2D, ref m_Texture2DEncoding, ref m_Texture2DIsDirty, doConversion: true);
            return m_Texture2D;
        }

        void TextureRefresh(ref Texture2D texture, ref string encoding, ref bool isDirty, bool doConversion)
        {
            if (texture == null || encoding != message.encoding || (m_Debayer && encoding.StartsWith("bayer")))
            {
                // texture is incompatible, we'll have to make a new one; so destroy the old one now
                if (texture != null)
                    Destroy(texture);

                if (doConversion)
                    texture = message.ToTexture2D(debayer: m_Debayer, flipY: m_Factory.m_FlipVertical);
                else
                    texture = message.ToTexture2D(debayer: false, convertBGR: false, flipY: false);

                encoding = message.encoding;
                isDirty = false;
            }
            else if (isDirty)
            {
                if (texture.width != message.width || texture.height != message.height)
                {
                    texture.Resize((int)message.width, (int)message.height);
                }

                byte[] convertedData = (doConversion) ? MessageExtensions.EncodingConversion(message, message.EncodingRequiresBGRConversion(), flipY: true) : message.data;

                texture.LoadRawTextureData(convertedData);
                texture.Apply();
                isDirty = false;
            }
        }

        public bool IsDrawingEnabled => false;
        public void SetDrawingEnabled(bool enabled) { }
        public void Redraw() { }
    }
}
