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
package se.hirt.searobots.engine;

import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.ships.DefaultAttackSub;
import se.hirt.searobots.engine.ships.SubmarineDrone;
import se.hirt.searobots.engine.ships.TargetDrone;
import se.hirt.searobots.engine.ships.claude.ClaudeAttackSub;
import se.hirt.searobots.engine.ships.codex.CodexAttackSub;
import se.hirt.searobots.engine.ships.gemini.GeminiAttackSub;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.Modifier;
import java.net.MalformedURLException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.ServiceLoader;
import java.util.function.Supplier;

/**
 * Resolves built-in and external {@link SubmarineController} implementations by short alias
 * ({@code claude}, {@code codex}, {@code gemini}, {@code default}, {@code drone}, {@code target})
 * or fully-qualified class name, with optional extra classpath entries for sparring across separate
 * submarine repositories.
 */
public final class ControllerLoader {

	public record ResolvedController(String displayName, String shortName, String className,
			Supplier<SubmarineController> factory, VehicleConfig vehicleConfig) {
	}

	private static final List<ResolvedController> BUILT_INS = List.of(
			builtIn("Codex", "CO", CodexAttackSub.class, CodexAttackSub::new, VehicleConfig.submarine()),
			builtIn("Claude", "CL", ClaudeAttackSub.class, ClaudeAttackSub::new, VehicleConfig.submarine()),
			builtIn("Gemini", "GE", GeminiAttackSub.class, GeminiAttackSub::new, VehicleConfig.submarine()),
			builtIn("Default", "DF", DefaultAttackSub.class, DefaultAttackSub::new, VehicleConfig.submarine()),
			builtIn("Sub Drone", "SD", SubmarineDrone.class, SubmarineDrone::new, VehicleConfig.submarine()),
			builtIn("Ship Drone (surface)", "TD", TargetDrone.class, TargetDrone::new, VehicleConfig.surfaceShip()));

	private ControllerLoader() {
	}

	private static ResolvedController builtIn(
		String displayName, String shortName, Class<? extends SubmarineController> cls,
		Supplier<SubmarineController> factory, VehicleConfig vehicleConfig) {
		return new ResolvedController(displayName, shortName, cls.getName(), factory, vehicleConfig);
	}

	/** Built-in combat submarines ordered roughly from baseline to strongest for gauntlets. */
	public static List<ResolvedController> allCombatBuiltIns() {
		return List.of(resolve("default"), resolve("gemini"), resolve("claude"), resolve("codex"));
	}

	/** All built-in ship options, including target drones. */
	public static List<ResolvedController> allBuiltIns() {
		return BUILT_INS;
	}

	/**
	 * Resolves a built-in alias/name or a fully-qualified {@link SubmarineController} class name
	 * using the context/application class loader.
	 */
	public static ResolvedController resolve(String spec) {
		return resolve(spec, ControllerLoader.class.getClassLoader());
	}

	/**
	 * Resolves a built-in alias/name or a fully-qualified {@link SubmarineController} class name
	 * using {@code classLoader}.
	 */
	public static ResolvedController resolve(String spec, ClassLoader classLoader) {
		if (spec == null || spec.isBlank()) {
			throw new IllegalArgumentException("Controller specification must not be blank");
		}
		String trimmed = spec.trim();
		String lower = trimmed.toLowerCase(Locale.ROOT);

		// 1. Exact built-in aliases
		ResolvedController aliasMatch = switch (lower) {
		case "codex", "codexattacksub" -> BUILT_INS.get(0);
		case "claude", "claudeattacksub" -> BUILT_INS.get(1);
		case "gemini", "geminiattacksub" -> BUILT_INS.get(2);
		case "default", "defaultattacksub" -> BUILT_INS.get(3);
		case "drone", "subdrone", "sub drone", "submarinedrone" -> BUILT_INS.get(4);
		case "target", "targetdrone", "ship", "ship drone" -> BUILT_INS.get(5);
		default -> null;
		};
		if (aliasMatch != null) {
			return aliasMatch;
		}

		// 2. Match built-in class name or display name substring (when spec has no dot)
		for (var b : BUILT_INS) {
			if (b.className().equals(trimmed)) {
				return b;
			}
		}
		if (!trimmed.contains(".")) {
			for (var b : BUILT_INS) {
				if (b.displayName().toLowerCase(Locale.ROOT).contains(lower)) {
					return b;
				}
			}
		}

		// 3. Load external SubmarineController class by FQCN
		ClassLoader loader = classLoader != null ? classLoader : ControllerLoader.class.getClassLoader();
		Class<?> raw;
		try {
			raw = Class.forName(trimmed, true, loader);
		} catch (ClassNotFoundException e) {
			throw new IllegalArgumentException(
					"Unknown built-in submarine or controller class not found on classpath: '" + trimmed + "'", e);
		}

		return fromClass(raw);
	}

	/**
	 * Validates and wraps a {@link SubmarineController} class with a public no-arg constructor.
	 */
	public static ResolvedController fromClass(Class<?> raw) {
		if (!SubmarineController.class.isAssignableFrom(raw)) {
			throw new IllegalArgumentException(
					raw.getName() + " does not implement " + SubmarineController.class.getName());
		}
		if (!Modifier.isPublic(raw.getModifiers()) || Modifier.isAbstract(raw.getModifiers())) {
			throw new IllegalArgumentException(raw.getName() + " must be a public, non-abstract class");
		}
		final Constructor<?> ctor;
		try {
			ctor = raw.getConstructor();
		} catch (NoSuchMethodException e) {
			throw new IllegalArgumentException(raw.getName() + " must declare a public no-arg constructor", e);
		}

		Supplier<SubmarineController> factory = () -> {
			try {
				return (SubmarineController) ctor.newInstance();
			} catch (ReflectiveOperationException e) {
				throw new IllegalStateException("Failed to instantiate controller " + raw.getName(), e);
			}
		};

		SubmarineController probe = factory.get();
		String name = probe.name();
		if (name == null || name.isBlank()) {
			name = raw.getSimpleName();
		}
		String trimmedName = name.trim();
		String shortName = SubmarineSnapshot.toShortName(probe.shortName(), trimmedName);
		return new ResolvedController(trimmedName, shortName, raw.getName(), factory, VehicleConfig.submarine());
	}

	/**
	 * Builds a child {@link URLClassLoader} over path-separator or comma-delimited JARs/directories
	 * (or returns {@code parent} if {@code extraCpSpec} is blank).
	 */
	public static ClassLoader createClassLoader(String extraCpSpec, ClassLoader parent) throws IOException {
		if (extraCpSpec == null || extraCpSpec.isBlank()) {
			return parent;
		}
		// Split on OS path separator or comma (avoid splitting Windows drive letters like C:\...)
		List<String> tokens = splitClasspathEntries(extraCpSpec);
		var urls = new ArrayList<URL>(tokens.size());
		for (String token : tokens) {
			Path p = Path.of(token).toAbsolutePath().normalize();
			if (!Files.exists(p)) {
				throw new IOException("Extra classpath entry does not exist: " + p);
			}
			try {
				urls.add(p.toUri().toURL());
			} catch (MalformedURLException e) {
				throw new IOException("Invalid classpath entry: " + p, e);
			}
		}
		return new URLClassLoader(urls.toArray(URL[]::new), parent);
	}

	/**
	 * Discovers any external {@link SubmarineController} implementations registered via
	 * {@code META-INF/services/se.hirt.searobots.api.SubmarineController}.
	 */
	public static List<ResolvedController> discoverServiceProviders(ClassLoader loader) {
		var discovered = new ArrayList<ResolvedController>();
		var sl = ServiceLoader.load(SubmarineController.class,
				loader != null ? loader : ControllerLoader.class.getClassLoader());
		for (var it = sl.iterator(); it.hasNext();) {
			try {
				SubmarineController instance = it.next();
				discovered.add(fromClass(instance.getClass()));
			} catch (Throwable t) {
				System.err.println("Skipping invalid SubmarineController service provider: " + t.getMessage());
			}
		}
		return discovered;
	}

	static List<String> splitClasspathEntries(String spec) {
		var out = new ArrayList<String>();
		// First split on commas, then on File.pathSeparator (handling Windows ';' vs Unix ':')
		for (String commaPart : spec.split(",")) {
			String trimmed = commaPart.trim();
			if (trimmed.isEmpty()) {
				continue;
			}
			if (File.pathSeparatorChar == ';') {
				for (String part : trimmed.split(";")) {
					if (!part.isBlank()) {
						out.add(part.trim());
					}
				}
			} else {
				for (String part : trimmed.split(":")) {
					if (!part.isBlank()) {
						out.add(part.trim());
					}
				}
			}
		}
		return out;
	}
}
