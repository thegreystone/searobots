#import "Common/ShaderLib/GLSLCompat.glsllib"
#import "Common/ShaderLib/Optics.glsllib"

uniform ENVMAP m_Texture;
uniform float m_Daylight;

varying vec3 direction;

void main() {
    vec4 color = Optics_GetEnvColor(m_Texture, normalize(direction));
    // Work in linear light; retain a faint cool horizon and cloud silhouette at night.
    color.rgb *= mix(vec3(0.0035, 0.0070, 0.0160), vec3(1.0), clamp(m_Daylight, 0.0, 1.0));
    gl_FragColor = color;
}
