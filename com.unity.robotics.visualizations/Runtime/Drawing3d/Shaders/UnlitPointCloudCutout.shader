Shader "Unlit/PointCloudCutout"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
    }
    
    SubShader
    {
        Tags {"Queue" = "AlphaTest" "IgnoreProjector" = "True"  "RenderType" = "TransparentCutout" }
        LOD 100
        ZWrite On
        Blend SrcAlpha OneMinusSrcAlpha

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 _MainTex_ST;

            struct appdata
            {
                float4 vertex : POSITION;
                float3 uvr : TEXCOORD0;
                float4 color : COLOR;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(
                    float4(v.vertex.x, v.vertex.y, v.vertex.z, 0)
                );

                float2 uv = v.uvr.xy;
                float radius = v.uvr.z;
                //If packing index into color.a: float2 uv = float2(round(v.color.a * 255 * 0.4), round(v.color.a * 255 % 2));
                o.vertex.x += (uv.x - 0.5) * 2 * radius * _ScreenParams.y / _ScreenParams.x;
#if UNITY_UV_STARTS_AT_TOP
                o.vertex.y -= (uv.y - 0.5) * 2 * radius;
#else
                o.vertex.y += (uv.y - 0.5) * 2 * radius;
#endif
                o.uv = uv;
                o.color = v.color;
                o.color.a = 1;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 color = tex2D(_MainTex, i.uv);
                clip(color.a-0.1);
                color.a = 1;
                return color * i.color;
            }
            ENDCG
        }
    }
}