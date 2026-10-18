#import "Common/ShaderLib/GLSLCompat.glsllib"

uniform sampler2D m_Texture;
uniform vec3 g_CameraPosition;
uniform float m_WaterHeight;
uniform float m_WaterDensity;
uniform float m_FogDensity;
uniform float m_SurfaceEnabled;
uniform float m_SurfaceDensity;

varying vec4 particleColor;
varying vec2 texCoord;
varying vec3 worldPosition;

void main() {
    vec4 particle = texture2D(m_Texture, texCoord) * particleColor;
    if (particle.a <= 0.001) {
        discard;
    }

    vec3 ray = worldPosition - g_CameraPosition;
    float rayLength = length(ray);
    float cameraDepth = m_WaterHeight - g_CameraPosition.y;
    float particleDepth = m_WaterHeight - worldPosition.y;

    // Fraction of this particular camera-to-particle segment below the water plane.
    // Both submerged: the whole path; opposite sides: only the submerged part.
    float waterFraction = clamp(max(cameraDepth, particleDepth)
            / max(abs(cameraDepth - particleDepth), 0.0001), 0.0, 1.0);
    float waterPath = rayLength * waterFraction;
    float fog = rayLength * m_FogDensity;

    // Attenuate the additive contribution, leaving the already fogged background intact.
    // Water removes red first, so distant white bubbles gently take on the water's blue tint.
    vec3 transmission = exp(-waterPath * m_WaterDensity * vec3(1.35, 1.0, 0.8));
    transmission *= exp(-fog * fog);

    // A crossing of the surface loses light at grazing angles. Rays entirely underwater
    // never get the sky's or waves' opacity, which was the original disappearing-wake bug.
    if (m_SurfaceEnabled > 0.5 && cameraDepth * particleDepth < 0.0) {
        float incidence = abs(ray.y) / max(rayLength, 0.0001);
        float reflection = 0.02 + 0.98 * pow(1.0 - incidence, 5.0);
        // Match the stronger absorption of a wake seen through the water surface.
        transmission *= (1.0 - reflection) * exp(-waterPath * m_SurfaceDensity);
    }

    gl_FragColor = vec4(particle.rgb * transmission, particle.a);
}