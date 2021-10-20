Shader "Unlit/PointCloudAdditive"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _Radius("Radius", float) = 0.1
    }
    
    SubShader
    {
        Tags {"Queue" = "Transparent" "IgnoreProjector" = "True"  "RenderType" = "Transparent" }
        LOD 100
        ZWrite Off
        Blend SrcAlpha One

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            float _Radius;
            sampler2D _MainTex;
            float4 _MainTex_ST;

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                float2 uv : TEXCOORD0;
                float4 color : COLOR;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(
                    float4(v.vertex.x, v.vertex.y, v.vertex.z, 0)
                );

                float2 uv = v.uv;
                //If packing index into color.a: float2 uv = float2(round(v.color.a * 255 * 0.4), round(v.color.a * 255 % 2));
                o.vertex.x += (uv.x - 0.5) * 2 * _Radius * _ScreenParams.y / _ScreenParams.x;
#if UNITY_UV_STARTS_AT_TOP
                o.vertex.y -= (uv.y - 0.5) * 2 * _Radius;
#else
                o.vertex.y += (uv.y - 0.5) * 2 * _Radius;
#endif
                o.uv = uv;
                o.color = v.color;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                return tex2D(_MainTex, i.uv) * i.color;
            }
            ENDCG
        }
    }
}
