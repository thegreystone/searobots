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
#extension GL_EXT_texture_array : enable
#import "Common/ShaderLib/GLSLCompat.glsllib"

#define ENABLE_PBRLightingUtils_getWorldPosition 1
#define ENABLE_PBRLightingUtils_getLocalPosition 1
#define ENABLE_PBRLightingUtils_getWorldNormal 1
#define ENABLE_PBRLightingUtils_getTexCoord 1
#define ENABLE_PBRLightingUtils_computeDirectLightContribution 1
#define ENABLE_PBRLightingUtils_computeProbesContribution 1

#define ENABLE_PBRTerrainUtils_readPBRTerrainLayers 1

#import "Common/ShaderLib/module/pbrlighting/PBRLightingUtils.glsllib"
#import "Common/MatDefs/Terrain/Modular/PBRTerrainUtils.glsllib"
#ifdef AFFLICTIONTEXTURE
    #import "Common/MatDefs/Terrain/Modular/AfflictionLib.glsllib"
#endif

//declare PBR Lighting vars
uniform vec4 g_LightData[NB_LIGHTS];
uniform vec3 g_CameraPosition;
uniform float m_SeabedAlbedoScale;
#ifdef SEABED_GRAVEL
    uniform float m_SeabedMapScale;

    float seabedHash(vec2 p) {
        return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
    }

    float seabedNoise(vec2 p) {
        vec2 cell = floor(p);
        vec2 blend = fract(p);
        blend = blend * blend * (3.0 - 2.0 * blend);
        return mix(mix(seabedHash(cell), seabedHash(cell + vec2(1.0, 0.0)), blend.x),
                   mix(seabedHash(cell + vec2(0.0, 1.0)), seabedHash(cell + vec2(1.0)), blend.x), blend.y);
    }
#endif

#ifdef DEBUG_VALUES_MODE
    uniform int m_DebugValuesMode;
#endif

#ifdef USE_FOG
    #import "Common/ShaderLib/MaterialFog.glsllib"
#endif

void main(){       
    vec3 wpos = PBRLightingUtils_getWorldPosition();
    vec3 worldViewDir = normalize(g_CameraPosition - wpos);
    
    // Create a blank PBRSurface.
    PBRSurface surface = PBRLightingUtils_createPBRSurface(worldViewDir);
    // jME 3.8.1's terrain path does not call readPBRSurface, which normally initializes these.
    surface.bakedLightContribution = vec3(0.0);
    surface.directLightContribution = vec3(0.0);
    surface.envLightContribution = vec3(0.0);
    surface.brightestNonGlobalLightStrength = 0.0;
    
    //pre-calculate necessary values for tri-planar blending
    TriPlanarUtils_calculateBlending(surface.geometryNormal);

    //reads terrain alphaMaps
    PBRTerrainUtils_readAlphaMaps();   
   
    //CUSTOM LIB EXAMPLE:
    #ifdef AFFLICTIONTEXTURE
        AfflictionLib_readAfflictionVector();
    #endif

    // read and blend up to 12 texture layers
    #for i=0..12 (#ifdef ALBEDOMAP_$i $0 #endif)
    
        PBRTerrainTextureLayer terrainTextureLayer_$i = PBRTerrainUtils_createAdvancedPBRTerrainLayer($i, surface.geometryNormal);
        
        #ifdef USE_FIRST_LAYER_AS_TRANSPARENCY
            if($i == 0){
                if(terrainTextureLayer_$i.blendValue > 0.01f){ 
                    discard;
                }     
            }    
        #endif 
        
        terrainTextureLayer_$i.roughness = m_Roughness_$i;
        terrainTextureLayer_$i.metallic = m_Metallic_$i;
        terrainTextureLayer_$i.emission = m_EmissiveColor_$i;

        #ifdef USE_TEXTURE_ARRAYS
            #if defined(TRI_PLANAR_MAPPING) || defined(TRI_PLANAR_MAPPING_$i)
              //triplanar for texture arrays:            
                PBRTerrainUtils_readTriPlanarAlbedoTexArray(m_AlbedoMap_$i, m_AlbedoMap_$i_scale, m_AlbedoTextureArray, terrainTextureLayer_$i);        
                #ifdef NORMALMAP_$i
                    PBRTerrainUtils_readTriPlanarNormalTexArray(m_NormalMap_$i, m_AlbedoMap_$i_scale, m_NormalParallaxTextureArray, terrainTextureLayer_$i);
                #endif
                #ifdef METALLICROUGHNESSMAP_$i
                    PBRTerrainUtils_readTriPlanarMetallicRoughnessAoEiTexArray(m_MetallicRoughnessMap_$i, m_AlbedoMap_$i_scale, m_MetallicRoughnessAoEiTextureArray, terrainTextureLayer_$i);
                #endif
            #else 
              //non tri-planar for texture arrays:
                PBRTerrainUtils_readAlbedoTexArray(m_AlbedoMap_$i, m_AlbedoMap_$i_scale, m_AlbedoTextureArray, terrainTextureLayer_$i);        
                #ifdef NORMALMAP_$i
                    PBRTerrainUtils_readNormalTexArray(m_NormalMap_$i, m_AlbedoMap_$i_scale, m_NormalParallaxTextureArray, terrainTextureLayer_$i);
                #endif
                #ifdef METALLICROUGHNESSMAP_$i
                    PBRTerrainUtils_readMetallicRoughnessAoEiTexArray(m_MetallicRoughnessMap_$i, m_AlbedoMap_$i_scale, m_MetallicRoughnessAoEiTextureArray, terrainTextureLayer_$i);
                #endif       
            #endif    
        #else
            #if defined(TRI_PLANAR_MAPPING) || defined(TRI_PLANAR_MAPPING_$i)
              //triplanar texture reads:    
                PBRTerrainUtils_readTriPlanarAlbedoTexture(m_AlbedoMap_$i, m_AlbedoMap_$i_scale, terrainTextureLayer_$i);        
                #ifdef NORMALMAP_$i
                    PBRTerrainUtils_readTriPlanarNormalTexture(m_NormalMap_$i, m_AlbedoMap_$i_scale, terrainTextureLayer_$i);
                #endif
                #ifdef METALLICROUGHNESSMAP_$i
                    PBRTerrainUtils_readTriPlanarMetallicRoughnessAoEiTexture(m_MetallicRoughnessMap_$i, m_AlbedoMap_$i_scale, terrainTextureLayer_$i);
                #endif
            #else 
              //non tri-planar texture reads:
                PBRTerrainUtils_readAlbedoTexture(m_AlbedoMap_$i, m_AlbedoMap_$i_scale, terrainTextureLayer_$i);        
                #ifdef NORMALMAP_$i
                    PBRTerrainUtils_readNormalTexture(m_NormalMap_$i, m_AlbedoMap_$i_scale, terrainTextureLayer_$i);
                #endif
                #ifdef METALLICROUGHNESSMAP_$i
                    PBRTerrainUtils_readMetallicRoughnessAoEiTexture(m_MetallicRoughnessMap_$i, m_AlbedoMap_$i_scale, terrainTextureLayer_$i);
                #endif       
            #endif            
        #endif
        
        #ifdef SEABED_GRAVEL
            if ($i == 2 && wpos.y < -1.0) {
                PBRTerrainTextureLayer gravel = PBRTerrainUtils_createAdvancedPBRTerrainLayer($i, surface.geometryNormal);
                PBRTerrainUtils_readTriPlanarAlbedoTexArray(4, m_SeabedMapScale, m_AlbedoTextureArray, gravel);
                PBRTerrainUtils_readTriPlanarNormalTexArray(3, m_SeabedMapScale, m_NormalParallaxTextureArray, gravel);
                PBRTerrainUtils_readTriPlanarMetallicRoughnessAoEiTexArray(3, m_SeabedMapScale, m_MetallicRoughnessAoEiTextureArray, gravel);

                // Broad, irregular deposits soften the repeated pebble clusters in the source tile.
                // World-space variation crosses terrain patch boundaries without a visible seam.
                float sediment = 0.12 + 0.28 * seabedNoise(wpos.xz / 47.0);
                gravel.albedo.rgb = mix(gravel.albedo.rgb, vec3(0.24, 0.22, 0.18), sediment);
                gravel.normal = normalize(mix(gravel.normal, surface.geometryNormal, sediment));
                gravel.roughness = mix(gravel.roughness, 0.93, sediment);
                gravel.ao = mix(gravel.ao, 1.0, sediment);
                float gravelBlend = smoothstep(1.0, 12.0, -wpos.y)
                                  * (0.68 + 0.27 * seabedNoise(wpos.xz / 31.0 + vec2(8.3, 19.7)));
                terrainTextureLayer_$i.albedo.rgb = mix(terrainTextureLayer_$i.albedo.rgb, gravel.albedo.rgb, gravelBlend);
                terrainTextureLayer_$i.normal = normalize(mix(terrainTextureLayer_$i.normal, gravel.normal, gravelBlend));
                terrainTextureLayer_$i.roughness = mix(terrainTextureLayer_$i.roughness, gravel.roughness, gravelBlend);
                terrainTextureLayer_$i.ao = mix(terrainTextureLayer_$i.ao, gravel.ao, gravelBlend);
            }
        #endif

        //CUSTOM LIB EXAMPLE: uses a custom alpha map to desaturate albedo color for a color-removal effect
        #ifdef AFFLICTIONTEXTURE
            afflictionMode = m_AfflictionMode_$i;     
            terrainTextureLayer_$i.albedo.rgb = alterLiveliness(terrainTextureLayer_$i.albedo.rgb, livelinessValue, afflictionMode); //changes saturation of albedo for this layer; does nothing if not using AfflictionAlphaMap for affliction splatting     
        #endif

        //blends this layer
        PBRTerrainUtils_blendPBRTerrainLayer(surface, terrainTextureLayer_$i);
    #endfor                     
    
    #ifdef DISCARD_ALPHA
        if(surface.alpha < m_AlphaDiscardThreshold){
            discard;
        }
    #endif   

    PBRLightingUtils_readSunLightExposureParams(surface);

    //CUSTOM LIB EXAMPLE: uses a custom alpha map and noise to blend an extra splat layer overtop of all other layers
    #ifdef AFFLICTIONTEXTURE
        AfflictionLib_blendSplatLayers(surface);
    #endif

    // Texture arrays are decoded to linear reflectance. Gradually lift submerged surfaces;
    // retain the original wet shoreline and all above-water colours, normals and occlusion.
    float seabedBlend = smoothstep(1.0, 12.0, -wpos.y);
    surface.albedo = min(surface.albedo * mix(1.0, m_SeabedAlbedoScale, seabedBlend), vec3(1.0));

   //Calculate necessary variables in pbr surface prior to applying lighting. Ensure all texture/param reading and blending occurrs prior to this being called!
    surface.frontFacing = gl_FrontFacing;
    surface.depth = gl_FragCoord.z;
    surface.NdotV = clamp(abs(dot(surface.normal, surface.viewDir)), 0.001, 1.0);
    PBRLightingUtils_calculatePreLightingValues(surface);
    
    // Calculate direct lights
    for(int i = 0;i < NB_LIGHTS; i+=3){
        vec4 lightData0 = g_LightData[i];
        vec4 lightData1 = g_LightData[i+1];
        vec4 lightData2 = g_LightData[i+2];    
        PBRLightingUtils_computeDirectLightContribution(
          lightData0, lightData1, lightData2, 
          surface
        );
    }    

    // Calculate env probes
    PBRLightingUtils_computeProbesContribution(surface);

    // Put it all together
    gl_FragColor.rgb = vec3(0.0);
    gl_FragColor.rgb += surface.bakedLightContribution;
    gl_FragColor.rgb += surface.directLightContribution;
    gl_FragColor.rgb += surface.envLightContribution;
    gl_FragColor.rgb += surface.emission;
    gl_FragColor.a = surface.alpha;  
    
    #ifdef USE_FOG
        gl_FragColor = MaterialFog_calculateFogColor(vec4(gl_FragColor));
    #endif
    
   //outputs the final value of the selected layer as a color for debug purposes. 
    #ifdef DEBUG_VALUES_MODE
        gl_FragColor = PBRLightingUtils_getColorOutputForDebugMode(m_DebugValuesMode, vec4(gl_FragColor.rgba), surface);
    #endif      
}
