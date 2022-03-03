Shader "Unlit/ImageGradient"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _Gradient("Gradient", 2D) = "white" {}
        _BrightnessMultiplier("BrightnessMultiplier", Float) = 1
		[Toggle] _flipY("Flip Y", Float) = 1
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
            sampler2D _Gradient;
            float4 _Gradient_ST;
            float _BrightnessMultiplier;
            float _flipY;

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
                float scalar = tex2D(_MainTex, i.uv).r * _BrightnessMultiplier;
                fixed4 color = tex2D(_Gradient, float2(scalar, 0));
                //color.b = 1;
                return color;
            }
			
            ENDCG
        }

	}
}
