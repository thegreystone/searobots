/*
 * Copyright (C) 2026 Marcus Hirt
 *
 * This software is free:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package se.hirt.searobots.viewer;

import com.jme3.asset.DesktopAssetManager;
import com.jme3.post.FilterPostProcessor;
import com.jme3.post.filters.FogFilter;
import com.jme3.post.filters.TranslucentBucketFilter;
import com.jme3.renderer.Camera;
import com.jme3.renderer.Caps;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.system.JmeSystem;
import com.jme3.system.NullRenderer;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Texture2D;
import org.junit.jupiter.api.Test;

import java.util.EnumSet;

import static org.junit.jupiter.api.Assertions.*;

class SubmarineScene3DPostProcessingTest {

	@Test
	void initializesAndResizesOnAppleOpenGLWithoutMultisampleSupport() {
		// OpenGL 3.2 alone does not guarantee either multisample capability.
		assertInitializesAndResizes(EnumSet.of(Caps.OpenGL32), 4, 1);
	}

	@Test
	void preservesRequestedSamplesWhenMultisampleTexturesAreSupported() {
		assertInitializesAndResizes(EnumSet.of(Caps.OpenGL32, Caps.FrameBufferMultisample, Caps.TextureMultisample), 4,
				4);
	}

	@Test
	void fallsBackWhenOnlyMultisampleFramebuffersAreSupported() {
		assertInitializesAndResizes(EnumSet.of(Caps.OpenGL32, Caps.FrameBufferMultisample), 4, 1);
	}

	@Test
	void initializesWithDefaultZeroSamplesEvenWhenMultisamplingIsSupported() {
		assertInitializesAndResizes(EnumSet.of(Caps.OpenGL32, Caps.FrameBufferMultisample, Caps.TextureMultisample), 0,
				1);
	}

	private static void assertInitializesAndResizes(EnumSet<Caps> caps, int requestedSamples, int expectedSamples) {
		NullRenderer renderer = new NullRenderer();
		renderer.getCaps().clear();
		renderer.getCaps().addAll(caps);
		renderer.initialize();
		DesktopAssetManager assets = new DesktopAssetManager(JmeSystem.getPlatformAssetConfigURL());
		FilterPostProcessor processor = SubmarineScene3D.createFilterPostProcessor(assets, renderer, requestedSamples);
		processor.addFilter(new FogFilter());
		processor.addFilter(new TranslucentBucketFilter());
		ViewPort viewPort = new ViewPort("post-processing regression", new Camera(640, 480));
		RenderManager renderManager = new RenderManager(renderer);

		// Exercise jME's real allocation and filter initialization, where the crash occurs.
		assertDoesNotThrow(() -> processor.initialize(renderManager, viewPort));
		try {
			assertTargets(processor, viewPort, 640, 480, expectedSamples);
			Texture2D originalSceneTexture = processor.getFilterTexture();
			Texture2D originalDepthTexture = processor.getDepthTexture();
			assertDoesNotThrow(() -> processor.reshape(viewPort, 800, 600));
			assertTargets(processor, viewPort, 800, 600, expectedSamples);
			assertNotSame(originalSceneTexture, processor.getFilterTexture());
			assertNotSame(originalDepthTexture, processor.getDepthTexture());
		} finally {
			processor.cleanup();
		}
	}

	private static void assertTargets(
		FilterPostProcessor processor, ViewPort viewPort, int width, int height, int expectedSamples) {
		assertEquals(expectedSamples, processor.getNumSamples());
		assertTexture(processor.getFilterTexture(), width, height, expectedSamples);
		assertTexture(processor.getDepthTexture(), width, height, expectedSamples);
		FrameBuffer frameBuffer = viewPort.getOutputFrameBuffer();
		assertNotNull(frameBuffer);
		assertEquals(width, frameBuffer.getWidth());
		assertEquals(height, frameBuffer.getHeight());
		assertEquals(expectedSamples, frameBuffer.getSamples());
	}

	private static void assertTexture(Texture2D texture, int width, int height, int expectedSamples) {
		assertNotNull(texture);
		assertEquals(width, texture.getImage().getWidth());
		assertEquals(height, texture.getImage().getHeight());
		assertEquals(expectedSamples, texture.getImage().getMultiSamples());
	}
}
