Shader "Unlit/VertexColorAlpha"
{
    Properties
    {
        _Fade ("Fade", Range(0.0,1.0)) = 1
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

            float _Fade;

            struct appdata
            {
                float4 vertex : POSITION;
                fixed4 color : COLOR;
            };

            struct v2f
            {
                float4 vertex : SV_POSITION;
                fixed4 vertexColor : COLOR;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.vertexColor = v.color;
                o.vertexColor.a *= _Fade;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                return i.vertexColor;
            }
            ENDCG
        }
    }
}
