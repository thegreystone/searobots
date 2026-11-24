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

import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.SubmarineController;
import se.hirt.searobots.api.VehicleConfig;
import se.hirt.searobots.engine.SimulationLoop.ControllerLimits;
import se.hirt.searobots.engine.ships.DefaultAttackSub;

import javax.tools.JavaCompiler;
import javax.tools.ToolProvider;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.lang.classfile.ClassFile;
import java.lang.classfile.ClassModel;
import java.lang.classfile.FieldModel;
import java.lang.classfile.MethodModel;
import java.lang.classfile.constantpool.ClassEntry;
import java.lang.classfile.constantpool.MemberRefEntry;
import java.lang.classfile.constantpool.MethodHandleEntry;
import java.lang.classfile.constantpool.PoolEntry;
import java.lang.constant.ClassDesc;
import java.lang.reflect.AccessFlag;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.URLClassLoader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;
import java.util.jar.JarOutputStream;

/**
 * Local developer pre-flight checker and registration manifest generator for the SeaRobots Monthly
 * Knockout League.
 * <ol>
 * <li>Compiles all {@code .java} files under {@code src/main/java} directly with
 * {@code javac --release 25 -proc:none} against {@code searobots-api} alone (mirroring league
 * compilation without Maven).</li>
 * <li>Runs a public JDK 25 {@code java.lang.classfile} bytecode rules check catching forbidden APIs
 * (reflection, I/O, networking, threads, {@code System.out}/{@code exit}/{@code getenv}, engine
 * internals, native methods) locally before opening a registration PR.</li>
 * <li>Optionally runs a 2-seed guarded smoke match under
 * {@link ControllerLimits#leagueDefaults()}.</li>
 * <li>Optionally prints a ready-to-paste {@code entries/<slug>.json} registration manifest
 * populated from the controller class and local git metadata.</li>
 * </ol>
 */

public final class LeaguePreflight {

	private static final long[] SMOKE_SEEDS = {0x2cL, 0x13L};
	private static final int SMOKE_DURATION_TICKS = 5 * 60 * 50;
	private static final String API_PREFIX = "se/hirt/searobots/api/";

	private static final Set<String> ALLOWED_JAVA_LANG = Set.of("java/lang/Object", "java/lang/String",
			"java/lang/StringBuilder", "java/lang/StringBuffer", "java/lang/AbstractStringBuilder",
			"java/lang/CharSequence", "java/lang/Appendable", "java/lang/Comparable", "java/lang/Iterable",
			"java/lang/Cloneable", "java/lang/AutoCloseable", "java/lang/Math", "java/lang/StrictMath",
			"java/lang/Number", "java/lang/Byte", "java/lang/Short", "java/lang/Integer", "java/lang/Long",
			"java/lang/Float", "java/lang/Double", "java/lang/Boolean", "java/lang/Character", "java/lang/Void",
			"java/lang/Enum", "java/lang/Record", "java/lang/System", "java/lang/Class", "java/lang/Throwable",
			"java/lang/Exception", "java/lang/RuntimeException", "java/lang/Error", "java/lang/AssertionError",
			"java/lang/IllegalArgumentException", "java/lang/IllegalStateException",
			"java/lang/IndexOutOfBoundsException", "java/lang/ArrayIndexOutOfBoundsException",
			"java/lang/StringIndexOutOfBoundsException", "java/lang/NullPointerException",
			"java/lang/NumberFormatException", "java/lang/ArithmeticException",
			"java/lang/UnsupportedOperationException", "java/lang/ClassCastException",
			"java/lang/NegativeArraySizeException", "java/lang/MatchException",
			"java/lang/IncompatibleClassChangeError", "java/lang/NoSuchFieldError", "java/lang/Override",
			"java/lang/Deprecated", "java/lang/SuppressWarnings", "java/lang/FunctionalInterface",
			"java/lang/SafeVarargs");

	private static final Set<String> ALLOWED_SYSTEM_METHODS = Set.of("arraycopy", "nanoTime", "currentTimeMillis",
			"identityHashCode", "lineSeparator");

	private static final Set<String> ALLOWED_CLASS_METHODS = Set.of("desiredAssertionStatus", "getSimpleName",
			"getName", "getCanonicalName", "isInstance", "isEnum", "getEnumConstants", "cast", "isAssignableFrom");

	private static final Set<String> BLOCKED_OBJECT_METHODS = Set.of("wait", "notify", "notifyAll", "finalize");

	private static final Set<String> BLOCKED_JAVA_UTIL = Set.of("java/util/ServiceLoader", "java/util/Timer",
			"java/util/TimerTask", "java/util/Scanner", "java/util/ResourceBundle", "java/util/PropertyResourceBundle",
			"java/util/Properties");

	private static final Set<String> COMPILER_BOOTSTRAPS = Set.of("java/lang/invoke/LambdaMetafactory",
			"java/lang/invoke/StringConcatFactory", "java/lang/runtime/ObjectMethods",
			"java/lang/runtime/SwitchBootstraps");

	private static final Set<String> BOOTSTRAP_SUPPORT = Set.of("java/lang/invoke/MethodHandles",
			"java/lang/invoke/MethodHandles$Lookup", "java/lang/invoke/MethodType", "java/lang/invoke/CallSite",
			"java/lang/invoke/MethodHandle", "java/lang/invoke/TypeDescriptor",
			"java/lang/invoke/TypeDescriptor$OfField", "java/lang/invoke/TypeDescriptor$OfMethod",
			"java/lang/invoke/LambdaMetafactory", "java/lang/invoke/StringConcatFactory",
			"java/lang/runtime/ObjectMethods", "java/lang/runtime/SwitchBootstraps");

	public record PreflightReport(boolean passed, String displayName, String shortName, int sourceFiles, int classFiles,
			long smokeCpuMs, List<String> errors) {
	}

	private LeaguePreflight() {
	}

	public static void main(String[] args) throws Exception {
		String controllerClass = null;
		Path srcDir = Path.of("src", "main", "java");
		boolean runSmoke = true;
		boolean printManifest = false;
		String license = "MIT";
		String description = null;

		for (int i = 0; i < args.length; i++) {
			switch (args[i]) {
			case "--sub" -> controllerClass = args[++i];
			case "--src-dir" -> srcDir = Path.of(args[++i]);
			case "--no-smoke" -> runSmoke = false;
			case "--manifest" -> printManifest = true;
			case "--license" -> license = args[++i];
			case "--description" -> description = args[++i];
			default -> throw new IllegalArgumentException("Unknown argument: " + args[i]);
			}
		}

		if (controllerClass == null || controllerClass.isBlank()) {
			System.err.println(
					"Usage: LeaguePreflight --sub <FQCN> [--src-dir src/main/java] [--no-smoke] [--manifest] [--license MIT] [--description \"...\"]");
			System.exit(2);
		}

		PreflightReport report = check(srcDir, controllerClass, runSmoke);
		System.out.println("=".repeat(76));
		System.out.printf(Locale.ROOT, "LEAGUE PRE-FLIGHT: %s  (%s)%n", report.passed() ? "PASS" : "FAIL",
				controllerClass);
		System.out.println("=".repeat(76));
		System.out.printf(Locale.ROOT, "  Sources compiled (javac --release 25 -proc:none): %d files -> %d classes%n",
				report.sourceFiles(), report.classFiles());
		if (!report.displayName().isEmpty()) {
			System.out.printf(Locale.ROOT, "  Controller identity: name=\"%s\", shortName=\"%s\"%n",
					report.displayName(), report.shortName());
		}
		if (runSmoke) {
			System.out.printf(Locale.ROOT, "  Smoke test CPU (2x 5m guarded matches): %d ms%n", report.smokeCpuMs());
		}
		for (String err : report.errors()) {
			System.out.println("  ERROR: " + err);
		}

		if (printManifest && report.passed()) {
			System.out.println();
			printRegistrationManifest(Path.of("").toAbsolutePath(), controllerClass, report.displayName(),
					report.shortName(), license, description);
		}

		if (!report.passed()) {
			System.exit(1);
		}
	}

	/**
	 * Runs bare {@code javac --release 25 -proc:none}, public bytecode rules scan, controller
	 * contract validation, and optional guarded smoke matches.
	 */
	public static PreflightReport check(Path srcDir, String controllerClass, boolean runSmoke) throws IOException {
		var errors = new ArrayList<String>();
		if (!Files.isDirectory(srcDir)) {
			errors.add("Source directory not found: " + srcDir.toAbsolutePath());
			return new PreflightReport(false, "", "", 0, 0, 0L, List.copyOf(errors));
		}

		List<Path> javaFiles;
		try (var stream = Files.walk(srcDir)) {
			javaFiles = stream.filter(p -> Files.isRegularFile(p) && p.toString().endsWith(".java"))
					.sorted(Comparator.naturalOrder()).toList();
		}
		if (javaFiles.isEmpty()) {
			errors.add("No .java files found under " + srcDir.toAbsolutePath());
			return new PreflightReport(false, "", "", 0, 0, 0L, List.copyOf(errors));
		}

		Path tempOut = Files.createTempDirectory("searobots-preflight-");
		Path classesDir = tempOut.resolve("classes");
		Files.createDirectories(classesDir);

		Path apiClasspath = resolveApiOnlyClasspath(tempOut);
		String diagnostics = compileStandalone(javaFiles, apiClasspath, classesDir);
		if (diagnostics != null) {
			errors.add(
					"Standalone javac --release 25 -proc:none failed (only searobots-api is allowed):\n" + diagnostics);
			return new PreflightReport(false, "", "", javaFiles.size(), 0, 0L, List.copyOf(errors));
		}

		List<Path> classFiles;
		try (var stream = Files.walk(classesDir)) {
			classFiles = stream.filter(p -> Files.isRegularFile(p) && p.toString().endsWith(".class"))
					.sorted(Comparator.naturalOrder()).toList();
		}

		errors.addAll(scanBytecode(classFiles));

		String displayName = "";
		String shortName = "";
		long smokeCpuMs = 0L;

		if (errors.isEmpty()) {
			try (var loader = new URLClassLoader(new URL[] {classesDir.toUri().toURL()},
					SubmarineController.class.getClassLoader())) {
				var resolved = ControllerLoader.resolve(controllerClass, loader);
				displayName = resolved.displayName();
				shortName = resolved.shortName();

				if (displayName.length() < 2 || displayName.length() > 24) {
					errors.add("Controller name() must be 2-24 chars (got \"" + displayName + "\")");
				}
				if (!shortName.matches("^[A-Z0-9]{2}$")) {
					errors.add("Controller shortName() must be 2 uppercase alphanumeric chars [A-Z0-9] (got \""
							+ shortName + "\")");
				}

				if (runSmoke && errors.isEmpty()) {
					long totalCpuNanos = 0L;
					for (long seed : SMOKE_SEEDS) {
						var config = MatchConfig.withDefaults(seed).withMatchDurationTicks(SMOKE_DURATION_TICKS);
						var world = new WorldGenerator().generate(config);
						var sim = new SimulationLoop();
						sim.setSpeedMultiplier(1e9);
						sim.setControllerLimits(ControllerLimits.leagueDefaults());
						var forfeited = new AtomicBoolean(false);

						sim.run(world, List.of(resolved.factory().get(), new DefaultAttackSub()),
								List.of(VehicleConfig.submarine(), VehicleConfig.submarine()),
								new SimulationListener() {
									@Override
									public void onTick(
										long tick, List<SubmarineSnapshot> subs, List<TorpedoSnapshot> torps) {
										if (!subs.isEmpty() && subs.getFirst().forfeited()) {
											forfeited.set(true);
										}
									}

									@Override
									public void onMatchEnd() {
									}
								});

						totalCpuNanos += sim.controllerCpuNanos(0);
						String reason = sim.forfeitReason(0);
						if (!reason.isEmpty()) {
							errors.add("Smoke seed 0x" + Long.toHexString(seed) + " triggered forfeit: " + reason);
						} else if (forfeited.get()) {
							errors.add("Smoke seed 0x" + Long.toHexString(seed) + " forfeited (left battle area)");
						}
					}
					smokeCpuMs = totalCpuNanos / 1_000_000L;
				}
			} catch (Throwable t) {
				errors.add("Controller contract/smoke check failed: " + t.getMessage());
			}
		}

		return new PreflightReport(errors.isEmpty(), displayName, shortName, javaFiles.size(), classFiles.size(),
				smokeCpuMs, List.copyOf(errors));
	}

	private static String compileStandalone(List<Path> javaFiles, Path apiClasspath, Path classesDir) {
		JavaCompiler compiler = ToolProvider.getSystemJavaCompiler();
		if (compiler == null) {
			return "JDK compiler unavailable; run with a full JDK 25";
		}
		var args = new ArrayList<String>();
		args.add("--release");
		args.add("25");
		args.add("-proc:none");
		args.add("-Xlint:none");
		args.add("-encoding");
		args.add("UTF-8");
		args.add("-cp");
		args.add(apiClasspath.toAbsolutePath().toString());
		args.add("-d");
		args.add(classesDir.toAbsolutePath().toString());
		for (Path f : javaFiles) {
			args.add(f.toAbsolutePath().toString());
		}
		var out = new ByteArrayOutputStream();
		var err = new ByteArrayOutputStream();
		int rc = compiler.run(null, out, err, args.toArray(String[]::new));
		return rc == 0 ? null : (out.toString(StandardCharsets.UTF_8) + err.toString(StandardCharsets.UTF_8)).trim();
	}

	private static Path resolveApiOnlyClasspath(Path tempDir) throws IOException {
		try {
			Path codeSource = Path
					.of(SubmarineController.class.getProtectionDomain().getCodeSource().getLocation().toURI());
			if (!Files.isRegularFile(codeSource) || !codeSource.toString().endsWith(".jar")) {
				return codeSource;
			}
			boolean hasNonApi = false;
			try (var jf = new JarFile(codeSource.toFile())) {
				var entries = jf.entries();
				while (entries.hasMoreElements()) {
					String name = entries.nextElement().getName();
					if (name.endsWith(".class") && !name.startsWith(API_PREFIX)
							&& !name.endsWith("module-info.class")) {
						hasNonApi = true;
						break;
					}
				}
			}
			if (!hasNonApi) {
				return codeSource;
			}
			Path apiOnly = tempDir.resolve("searobots-api-only.jar");
			try (var jf = new JarFile(codeSource.toFile());
					var jos = new JarOutputStream(Files.newOutputStream(apiOnly))) {
				var entries = jf.entries();
				while (entries.hasMoreElements()) {
					JarEntry entry = entries.nextElement();
					if (!entry.isDirectory() && entry.getName().startsWith(API_PREFIX)
							&& entry.getName().endsWith(".class")) {
						jos.putNextEntry(new JarEntry(entry.getName()));
						try (var in = jf.getInputStream(entry)) {
							in.transferTo(jos);
						}
						jos.closeEntry();
					}
				}
			}
			return apiOnly;
		} catch (URISyntaxException e) {
			throw new IOException("Cannot resolve searobots-api classpath", e);
		}
	}

	static List<String> scanBytecode(List<Path> classFiles) throws IOException {
		var models = new ArrayList<ClassModel>(classFiles.size());
		var ownClasses = new HashSet<String>();
		var violations = new ArrayList<String>();
		ClassFile cf = ClassFile.of();

		for (Path file : classFiles) {
			ClassModel m = cf.parse(Files.readAllBytes(file));
			String name = m.thisClass().asInternalName();
			if (name.startsWith("java/") || name.startsWith("javax/") || name.startsWith("sun/")
					|| name.startsWith("jdk/") || name.startsWith("se/hirt/searobots/engine/")
					|| name.startsWith("se/hirt/searobots/api/")) {
				violations.add(name + ": class declared in reserved package");
			}
			ownClasses.add(name);
			models.add(m);
		}

		for (ClassModel m : models) {
			String thisClass = m.thisClass().asInternalName();
			for (FieldModel f : m.fields()) {
				checkDesc(thisClass, "field " + f.fieldName().stringValue(), f.fieldTypeSymbol(), ownClasses,
						violations);
			}
			for (MethodModel method : m.methods()) {
				String mName = method.methodName().stringValue();
				if (method.flags().has(AccessFlag.NATIVE)) {
					violations.add(thisClass + "." + mName + ": native methods are forbidden");
				}
				if ("finalize".equals(mName) && "()V".equals(method.methodType().stringValue())) {
					violations.add(thisClass + ".finalize(): finalizers are forbidden");
				}
				var desc = method.methodTypeSymbol();
				checkDesc(thisClass, "method " + mName, desc.returnType(), ownClasses, violations);
				for (int i = 0; i < desc.parameterCount(); i++) {
					checkDesc(thisClass, "method " + mName, desc.parameterType(i), ownClasses, violations);
				}
			}
			for (PoolEntry entry : m.constantPool()) {
				switch (entry) {
				case ClassEntry ce -> {
					String ref = stripArray(ce.asInternalName());
					if (ref != null && !isAllowed(ref, ownClasses, true)) {
						violations.add(thisClass + ": forbidden class reference '" + ref + "'");
					}
				}
				case MemberRefEntry mre -> checkMember(thisClass, stripArray(mre.owner().asInternalName()),
						mre.name().stringValue(), ownClasses, violations);
				case MethodHandleEntry mhe -> {
					String owner = stripArray(mhe.reference().owner().asInternalName());
					if (owner != null && !COMPILER_BOOTSTRAPS.contains(owner)) {
						checkMember(thisClass, owner, mhe.reference().name().stringValue(), ownClasses, violations);
					}
				}
				default -> {
				}
				}
			}
		}
		return violations;
	}

	private static void checkMember(
		String thisClass, String owner, String member, Set<String> ownClasses, List<String> violations) {
		if (owner == null || ownClasses.contains(owner) || COMPILER_BOOTSTRAPS.contains(owner)) {
			return;
		}
		if (!isAllowed(owner, ownClasses, false)) {
			violations.add(thisClass + ": forbidden member access '" + owner + "." + member + "'");
		} else if ("java/lang/System".equals(owner) && !ALLOWED_SYSTEM_METHODS.contains(member)) {
			violations.add(thisClass + ": forbidden System member 'java/lang/System." + member + "'");
		} else if ("java/lang/Class".equals(owner) && !ALLOWED_CLASS_METHODS.contains(member)) {
			violations.add(thisClass + ": forbidden Class method 'java/lang/Class." + member + "'");
		} else if ("java/lang/Object".equals(owner) && BLOCKED_OBJECT_METHODS.contains(member)) {
			violations.add(thisClass + ": forbidden Object method 'java/lang/Object." + member + "'");
		}
	}

	private static void checkDesc(
		String thisClass, String ctx, ClassDesc desc, Set<String> ownClasses, List<String> violations) {
		while (desc.isArray()) {
			desc = desc.componentType();
		}
		if (desc.isPrimitive()) {
			return;
		}
		String d = desc.descriptorString();
		if (d.startsWith("L") && d.endsWith(";")) {
			String internal = d.substring(1, d.length() - 1);
			if (!isAllowed(internal, ownClasses, false)) {
				violations.add(thisClass + " (" + ctx + "): forbidden type '" + internal + "'");
			}
		}
	}

	private static boolean isAllowed(String name, Set<String> ownClasses, boolean allowBootstrapSupport) {
		if (ownClasses.contains(name) || name.startsWith(API_PREFIX) || ALLOWED_JAVA_LANG.contains(name)) {
			return true;
		}
		if (allowBootstrapSupport && BOOTSTRAP_SUPPORT.contains(name)) {
			return true;
		}
		if (name.startsWith("java/util/")) {
			for (String b : BLOCKED_JAVA_UTIL) {
				if (name.equals(b) || name.startsWith(b + "$")) {
					return false;
				}
			}
			String rest = name.substring("java/util/".length());
			return !rest.contains("/") || name.startsWith("java/util/function/") || name.startsWith("java/util/stream/")
					|| name.startsWith("java/util/random/");
		}
		return name.startsWith("java/math/") || name.startsWith("java/time/");
	}

	private static String stripArray(String name) {
		if (name == null || name.isEmpty()) {
			return null;
		}
		if (!name.startsWith("[")) {
			return name;
		}
		int i = 0;
		while (i < name.length() && name.charAt(i) == '[') {
			i++;
		}
		return (i < name.length() && name.charAt(i) == 'L' && name.endsWith(";"))
				? name.substring(i + 1, name.length() - 1) : null;
	}

	static String formatManifestJson(
		String slug, String name, String shortName, String githubUser, String repoUrl, String commitSha,
		String controllerClass, String license, String description) {
		return """
				{
				  "slug": "%s",
				  "name": "%s",
				  "shortName": "%s",
				  "githubUser": "%s",
				  "repo": "%s",
				  "commit": "%s",
				  "controllerClass": "%s",
				  "license": "%s",
				  "description": "%s"
				}""".formatted(esc(slug), esc(name), esc(shortName), esc(githubUser), esc(repoUrl), esc(commitSha),
				esc(controllerClass), esc(license), esc(description));
	}

	private static void printRegistrationManifest(
		Path repoRoot, String controllerClass, String name, String shortName, String license, String description) {
		String commit = runGit(repoRoot, "rev-parse", "HEAD");
		if (commit.isEmpty()) {
			commit = "0000000000000000000000000000000000000000";
		}
		String remote = normalizeGithubHttps(runGit(repoRoot, "remote", "get-url", "origin"));
		String githubUser = extractGithubUser(remote);
		String slug = name.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]+", "-").replaceAll("^-|-$", "");
		if (slug.length() < 3) {
			slug = "my-sub";
		}
		String desc = description != null && !description.isBlank() ? description
				: "Autonomous SeaRobots submarine controller.";
		String dirty = runGit(repoRoot, "status", "--porcelain");

		System.out.println("Suggested registration manifest (entries/" + slug + ".json):");
		System.out.println(
				formatManifestJson(slug, name, shortName, githubUser, remote, commit, controllerClass, license, desc));
		if (!dirty.isEmpty()) {
			System.out.println(
					"\nWARNING: You have uncommitted local changes - commit and push before submitting this SHA!");
		}
	}

	static String normalizeGithubHttps(String remote) {
		if (remote == null || remote.isBlank()) {
			return "https://github.com/YOUR_GITHUB_USER/YOUR_REPO";
		}
		String s = remote.trim();
		if (s.startsWith("git@github.com:")) {
			s = "https://github.com/" + s.substring("git@github.com:".length());
		}
		if (s.endsWith(".git")) {
			s = s.substring(0, s.length() - 4);
		}
		return s;
	}

	static String extractGithubUser(String httpsRepo) {
		String prefix = "https://github.com/";
		if (httpsRepo != null && httpsRepo.startsWith(prefix)) {
			String rest = httpsRepo.substring(prefix.length());
			int slash = rest.indexOf('/');
			if (slash > 0) {
				return rest.substring(0, slash);
			}
		}
		return "YOUR_GITHUB_USER";
	}

	private static String runGit(Path dir, String ... gitArgs) {
		var cmd = new ArrayList<String>(gitArgs.length + 1);
		cmd.add("git");
		cmd.addAll(List.of(gitArgs));
		try {
			Process p = new ProcessBuilder(cmd).directory(dir.toFile()).redirectErrorStream(true).start();
			String out = new String(p.getInputStream().readAllBytes(), StandardCharsets.UTF_8).trim();
			if (p.waitFor(5, TimeUnit.SECONDS) && p.exitValue() == 0) {
				return out;
			}
		} catch (Exception ignored) {
		}
		return "";
	}

	private static String esc(String s) {
		return s == null ? "" : s.replace("\\", "\\\\").replace("\"", "\\\"");
	}
}
