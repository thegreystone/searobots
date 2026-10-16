#import "Common/ShaderLib/GLSLCompat.glsllib"

#define ENABLE_PBRLightingUtils_getWorldPosition 1
#define ENABLE_PBRLightingUtils_getWorldNormal 1
#define ENABLE_PBRLightingUtils_getWorldTangent 1
#define ENABLE_PBRLightingUtils_getTexCoord 1
#define ENABLE_PBRLightingUtils_readPBRSurface 1
#define ENABLE_PBRLightingUtils_computeDirectLightContribution 1
#import "Common/ShaderLib/module/pbrlighting/PBRLightingUtils.glsllib"

uniform vec3 g_CameraPosition;
uniform bool m_BakeNormals;

void main() {
    vec3 viewDirection = normalize(g_CameraPosition - PBRLightingUtils_getWorldPosition());
    PBRSurface surface = PBRLightingUtils_createPBRSurface(viewDirection);
    PBRLightingUtils_readPBRSurface(surface);
    // Store surface data only: the runtime impostor receives the same lighting as the model.
    gl_FragColor = vec4(m_BakeNormals ? surface.normal * 0.5 + 0.5 : surface.albedo, surface.alpha);
}
