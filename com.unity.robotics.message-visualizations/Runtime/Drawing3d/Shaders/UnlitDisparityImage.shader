Shader "Unlit/DisparityImage"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _MinDisparity("Min Disparity", float) = 1
        _MaxDisparity("Max Disparity", float) = 10
        _Color0 ("Color 0", Color) = (1,1,1,1)
        _Color100 ("Color 100", Color) = (0,0,0,1)
    }
    SubShader
    {
        Tags {"Queue" = "Transparent" "IgnoreProjector" = "True"  "RenderType"="Transparent" }
        LOD 100
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            float _MinDisparity;
            float _MaxDisparity;
            fixed4 _Color0;
            fixed4 _Color100;

            sampler2D _MainTex;
            float4 _MainTex_ST;

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float2 uv : TEXCOORD0;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float raw = tex2D(_MainTex, i.uv).r;
                float scaled = (raw - _MinDisparity) / (_MaxDisparity - _MinDisparity);
                return lerp(_Color0, _Color100, scaled);
            }
            ENDCG
        }
    }
}
