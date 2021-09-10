Shader "Unlit/ImageMsg"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
		[Toggle] _convertBGR("convert BGR", Float) = 0
		[Toggle] _flipY("Flip Y", Float) = 1
		[Toggle] _gray("Gray", Float) = 0
    }
	
	SubShader
	{
		Tags { "Queue"="Overlay" "IgnoreProjector"="True" "RenderType"="Transparent" }
		Lighting Off
		Cull Off
		ZTest Always
		ZWrite Off
		Fog { Mode Off }
		
		Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag

            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 _MainTex_ST;
			float _convertBGR;
			float _flipY;
			float _gray;

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

            v2f vert(appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(
                    float4(v.vertex.x, v.vertex.y, v.vertex.z, 0)
                );
				o.uv = _flipY == 0? v.uv: float2(v.uv.x, 1-v.uv.y);
				return o;
			}

            fixed4 frag (v2f i) : SV_Target
            {
                fixed4 color = tex2D(_MainTex, i.uv);
				if(_gray > 0.5)
					color = color.rrra;
				else if(_convertBGR > 0.5)
					color = color.bgra;
                color.a = 1;
                return color;
            }
			
            ENDCG
        }

	}
}
