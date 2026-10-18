#import "Common/ShaderLib/GLSLCompat.glsllib"

uniform mat4 g_WorldViewProjectionMatrix;
uniform mat4 g_WorldMatrix;

attribute vec3 inPosition;
attribute vec4 inColor;
attribute vec4 inTexCoord;

varying vec4 particleColor;
varying vec2 texCoord;
varying vec3 worldPosition;

void main() {
    vec4 position = vec4(inPosition, 1.0);
    gl_Position = g_WorldViewProjectionMatrix * position;
    worldPosition = (g_WorldMatrix * position).xyz;
    particleColor = inColor;
    texCoord = inTexCoord.xy;
}