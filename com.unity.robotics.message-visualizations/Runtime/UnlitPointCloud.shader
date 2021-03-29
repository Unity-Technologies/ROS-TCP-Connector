Shader "Unlit/PointCloud"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _Radius("Radius", float) = 0.1
    }
    
    SubShader
    {
        Tags {"Queue" = "AlphaTest" "IgnoreProjector" = "True"  "RenderType" = "TransparentCutout" }
        LOD 100
        ZWrite On
        AlphaTest Greater 0.5
        Blend SrcAlpha OneMinusSrcAlpha

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

                o.vertex.x += (v.uv.x - 0.5) * 2 * _Radius * _ScreenParams.y / _ScreenParams.x;
                o.vertex.y -= (v.uv.y - 0.5) * 2 * _Radius;
                //o.vertex.x += v.uv.x*0.1;
                //o.vertex.y += v.uv.y*0.1;
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                o.color = v.color;
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
