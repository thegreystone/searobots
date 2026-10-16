/*
 * Copyright (c) 2009-2019 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#import "Common/ShaderLib/GLSLCompat.glsllib"
#import "Common/ShaderLib/Instancing.glsllib"
#import "Common/ShaderLib/Skinning.glsllib"
#import "Common/ShaderLib/MorphAnim.glsllib"

uniform vec4 m_BaseColor;

uniform vec4 g_AmbientLightColor;
varying vec2 texCoord;

#ifdef SEPARATE_TEXCOORD
   varying vec2 texCoord2;
#endif

#if defined(SEPARATE_TEXCOORD) || defined(IMPOSTOR_BACK_VIEWS)
    attribute vec2 inTexCoord2;
#endif

uniform vec3 g_CameraPosition;
#ifdef IMPOSTOR_DISTANCE_FADE
    uniform vec2 m_ImpostorDistanceFadeRange;
#endif
#ifdef IMPOSTOR_TOP_VIEWS
    uniform vec2 m_ImpostorTopFadeRange;
#endif
varying vec4 Color;

attribute vec3 inPosition;
attribute vec2 inTexCoord;
attribute vec3 inNormal;

#if defined (VERTEX_COLOR) || defined(USE_VERTEX_COLORS_AS_SUN_INTENSITY)
    attribute vec4 inColor;
#endif

#if defined(USE_VERTEX_COLORS_AS_SUN_INTENSITY)
    varying vec4 vertColors;
#endif

varying vec3 wNormal;
varying vec3 wPosition;

varying vec3 lPosition;

attribute vec4 inTangent;
varying vec4 wTangent;

#ifdef USE_FOG
    varying float fogDistance;
#endif

void main(){    
    
    vec4 modelSpacePos = vec4(inPosition, 1.0);
    vec3 modelSpaceNorm = inNormal;
    vec3 modelSpaceTan  = inTangent.xyz;
    
    lPosition = modelSpacePos.xyz;
    
    
    #ifdef USE_VERTEX_COLORS_AS_SUN_INTENSITY
        vertColors = inColor;
    #endif

    
    #ifdef NUM_MORPH_TARGETS
         #if defined(NORMALMAP) && !defined(VERTEX_LIGHTING)
            Morph_Compute(modelSpacePos, modelSpaceNorm, modelSpaceTan);
         #else
            Morph_Compute(modelSpacePos, modelSpaceNorm);
         #endif
    #endif

    #ifdef NUM_BONES
         #if defined(NORMALMAP) && !defined(VERTEX_LIGHTING)
         Skinning_Compute(modelSpacePos, modelSpaceNorm, modelSpaceTan);
         #else
         Skinning_Compute(modelSpacePos, modelSpaceNorm);
         #endif
    #endif

    gl_Position = TransformWorldViewProjection(modelSpacePos);
    texCoord = inTexCoord;
    #ifdef SEPARATE_TEXCOORD
       texCoord2 = inTexCoord2;
    #endif

    wPosition = TransformWorld(modelSpacePos).xyz;
    wNormal  = TransformWorldNormal(modelSpaceNorm);
    
    wTangent = vec4(TransformWorldNormal(modelSpaceTan),inTangent.w);

    #ifdef IMPOSTOR_BACK_VIEWS
        // Opposite views were captured separately from the same asymmetric modeled tree.
        // Flip both N and T: B remains upward while texture U follows the rear camera's right.
        if (dot(wNormal, g_CameraPosition - wPosition) < 0.0) {
            texCoord = inTexCoord2;
            wNormal = -wNormal;
            wTangent.xyz = -wTangent.xyz;
        }
    #endif

    Color = m_BaseColor;
    
    #ifdef VERTEX_COLOR                
        Color *= inColor;
    #endif

    #ifdef IMPOSTOR_DISTANCE_FADE
        // Fade within the ground's clipping range, before whole card regions are culled.
        float treeDistance = distance(g_CameraPosition, wPosition);
        Color.a *= 1.0 - smoothstep(m_ImpostorDistanceFadeRange.x, m_ImpostorDistanceFadeRange.y, treeDistance);
    #endif

    #ifdef IMPOSTOR_TOP_VIEWS
        // The crown view fills overhead silhouettes. At low angles its proxy slope would
        // stretch needles and palm fronds into a cone across the accurate side views.
        float elevation = normalize(g_CameraPosition - wPosition).y;
        Color.a *= smoothstep(m_ImpostorTopFadeRange.x, m_ImpostorTopFadeRange.y, elevation);
    #endif
    
    #ifdef USE_FOG
        fogDistance = distance(g_CameraPosition, (TransformWorld(modelSpacePos)).xyz);
    #endif

}
