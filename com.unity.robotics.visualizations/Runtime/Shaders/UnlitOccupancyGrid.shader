Shader "Unlit/OccupancyGrid"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _ColorUnknown ("Unknown Color", Color) = (0,0,0,0)
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
            Offset -1, -1

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            fixed4 _ColorUnknown;
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
                // byte range 0-255 is rescaled down to 0-1 here; for an occupancy grid we want 100 to rescale to 1, so we need a multiplier.
                float frac = tex2D(_MainTex, i.uv).r * 255.0/100.0;
                if (frac > 1)
                    return _ColorUnknown;
                else
                    return lerp(_Color0, _Color100, frac);
            }
            ENDCG
        }
    }
}
