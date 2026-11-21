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
package se.hirt.searobots.viewer.showreel;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import javax.imageio.ImageIO;

import com.jme3.post.SceneProcessor;
import com.jme3.profile.AppProfiler;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.texture.FrameBuffer;
import com.jme3.texture.Image;
import com.jme3.util.BufferUtils;

/**
 * Captures finished frames, overlays included, and encodes them. Add it to the last view port (the
 * GUI's); each call to {@link #captureNextFrame()} grabs the frame being rendered. Frames are
 * streamed as raw RGBA to an {@code ffmpeg} process (H.264 MP4), or written as numbered PNG files
 * when there is no ffmpeg. Encoding runs on its own thread; the render thread only waits when it
 * gets several frames ahead.
 */
public final class FrameRecorder implements SceneProcessor {
	private static final byte[] END = new byte[0];

	private final Path output;
	private final String ffmpeg; // null: write PNG frames into output
	private final int fps, crf;
	private final BlockingQueue<byte[]> queue = new ArrayBlockingQueue<>(4);
	private RenderManager renderManager;
	private ByteBuffer readBuffer;
	private int width, height;
	private boolean capture, initialized;
	private Thread writer;
	private Process process;
	private volatile Exception failure;
	private long frames;

	/**
	 * @param output
	 *            the video file, or the directory for PNG frames if {@code ffmpeg} is null
	 * @param ffmpeg
	 *            the ffmpeg executable, or null
	 */
	public FrameRecorder(Path output, String ffmpeg, int fps, int crf) {
		this.output = output;
		this.ffmpeg = ffmpeg;
		this.fps = fps;
		this.crf = crf;
	}

	/**
	 * The ffmpeg executable: {@code configured} if given, else {@code FFMPEG}, else one on the
	 * PATH.
	 */
	public static String findFfmpeg(String configured) {
		if (configured != null && !configured.isBlank())
			return configured;
		String env = System.getenv("FFMPEG");
		if (env != null && !env.isBlank() && Files.isExecutable(Path.of(env)))
			return env;
		String path = System.getenv("PATH");
		if (path == null)
			return null;
		boolean windows = System.getProperty("os.name", "").toLowerCase(Locale.ROOT).contains("win");
		for (String dir : path.split(File.pathSeparator)) {
			Path candidate = Path.of(dir.isBlank() ? "." : dir, windows ? "ffmpeg.exe" : "ffmpeg");
			if (Files.isExecutable(candidate))
				return candidate.toString();
		}
		return null;
	}

	/** Grabs the frame currently being rendered once it is complete. */
	public void captureNextFrame() {
		capture = true;
	}

	public long frames() {
		return frames;
	}

	/** Throws if encoding failed, so a broken recording stops instead of rendering on. */
	public void checkHealthy() throws IOException {
		if (failure != null)
			throw new IOException("Frame encoding failed: " + failure, failure);
	}

	/** Finishes encoding and waits for the file to be complete. */
	public void finish() throws IOException, InterruptedException {
		if (writer == null)
			return;
		queue.put(END);
		writer.join();
		if (process != null) {
			int exit = process.waitFor();
			if (exit != 0 && failure == null)
				failure = new IOException("ffmpeg exited with " + exit + "; see " + logFile());
		}
		checkHealthy();
	}

	private Path logFile() {
		return output.resolveSibling(output.getFileName() + ".ffmpeg.log");
	}

	@Override
	public void initialize(RenderManager rm, ViewPort vp) {
		renderManager = rm;
		initialized = true;
	}

	@Override
	public void reshape(ViewPort vp, int w, int h) {
		if (writer != null && (w != width || h != height))
			failure = new IllegalStateException("The window was resized during recording, to " + w + "x" + h);
	}

	@Override
	public boolean isInitialized() {
		return initialized;
	}

	@Override
	public void preFrame(float tpf) {
	}

	@Override
	public void postQueue(RenderQueue rq) {
	}

	@Override
	public void postFrame(FrameBuffer out) {
		if (!capture || failure != null)
			return;
		capture = false;
		var camera = renderManager.getCurrentCamera();
		int w = camera.getWidth(), h = camera.getHeight();
		try {
			if (writer == null)
				start(w, h);
			readBuffer.clear();
			renderManager.getRenderer().readFrameBufferWithFormat(out, readBuffer, Image.Format.RGBA8);
			byte[] frame = new byte[w * h * 4];
			readBuffer.rewind();
			readBuffer.get(frame);
			queue.put(frame);
			frames++;
		} catch (IOException | RuntimeException e) {
			failure = e;
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			failure = e;
		}
	}

	private void start(int w, int h) throws IOException {
		width = w;
		height = h;
		readBuffer = BufferUtils.createByteBuffer(w * h * 4);
		OutputStream sink;
		if (ffmpeg != null) {
			Files.createDirectories(output.toAbsolutePath().getParent());
			List<String> command = new ArrayList<>(List.of(ffmpeg, "-hide_banner", "-loglevel", "warning", "-y", "-f",
					"rawvideo", "-pix_fmt", "rgba", "-s", w + "x" + h, "-framerate", Integer.toString(fps), "-i", "-",
					"-vf", "vflip", "-c:v", "libx264", "-preset", "slow", "-crf", Integer.toString(crf), "-pix_fmt",
					"yuv420p", "-movflags", "+faststart", output.toString()));
			process = new ProcessBuilder(command).redirectErrorStream(true)
					.redirectOutput(ProcessBuilder.Redirect.to(logFile().toFile())).start();
			sink = process.getOutputStream();
		} else {
			Files.createDirectories(output);
			sink = null;
		}
		writer = Thread.ofPlatform().name("showreel-encoder").start(() -> encode(sink));
	}

	private void encode(OutputStream sink) {
		long index = 0;
		try (sink) {
			for (byte[] frame; (frame = queue.take()) != END;) {
				if (failure != null)
					continue; // drain so the render thread never blocks on a dead encoder
				if (sink != null)
					sink.write(frame);
				else
					writePng(frame, output.resolve(String.format(Locale.ROOT, "frame-%05d.png", index)));
				index++;
			}
		} catch (IOException | RuntimeException e) {
			failure = e;
			queue.clear();
		} catch (InterruptedException e) {
			failure = e;
		}
	}

	private void writePng(byte[] rgba, Path file) throws IOException {
		var image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
		for (int y = 0; y < height; y++) {
			int row = (height - 1 - y) * width * 4; // OpenGL rows run bottom-up
			for (int x = 0; x < width; x++) {
				int i = row + x * 4;
				image.setRGB(x, y, (rgba[i] & 0xff) << 16 | (rgba[i + 1] & 0xff) << 8 | (rgba[i + 2] & 0xff));
			}
		}
		ImageIO.write(image, "png", file.toFile());
	}

	@Override
	public void cleanup() {
	}

	@Override
	public void setProfiler(AppProfiler profiler) {
	}
}
