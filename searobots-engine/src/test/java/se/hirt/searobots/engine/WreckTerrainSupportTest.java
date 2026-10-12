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

import jdk.jfr.Recording;
import jdk.jfr.consumer.RecordedEvent;
import jdk.jfr.consumer.RecordingFile;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import se.hirt.searobots.api.CurrentField;
import se.hirt.searobots.api.CurrentField.CurrentBand;
import se.hirt.searobots.api.MatchConfig;
import se.hirt.searobots.api.TerrainMap;
import se.hirt.searobots.api.Vec2;
import se.hirt.searobots.api.Vec3;
import se.hirt.searobots.api.VehicleConfig;

import java.awt.Color;
import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class WreckTerrainSupportTest {

	private static final long SLOW_SEED = -451325398205793679L;
	private static final long SEAM_SEED = 6437323593578695347L;
	private static final double TICK_SECONDS = 0.02;

	@Test
	void unevenSeabedSupportConsumesTheWholeTickAndPreservesDriftAndRotation(@TempDir
	Path directory) throws IOException {
		var config = MatchConfig.withDefaults(SLOW_SEED);
		var terrain = slowSeabed();
		var currents = slowCurrents();
		var wreck = supportedWreck();
		var start = wreck.pose();
		var physics = new SubmarinePhysics(config);
		Path file = directory.resolve("wreck-support.jfr");
		try (var recording = new Recording()) {
			recording.enable(SimulationJfrEvents.TERRAIN_EVENT_NAME).withThreshold(Duration.ZERO).withoutStackTrace();
			recording.start();
			physics.step(wreck, TICK_SECONDS, terrain, currents, config.battleArea(), 0);
			assertTrue(wreck.pose().position().subtract(start.position()).length() > 0.004,
					"A supported wreck must retain its permitted drift throughout the tick");
			assertTrue(wreck.heading() - start.heading() > 0.00005,
					"Support must not discard most of the remaining yaw motion");
			assertTrue(wreck.pitch() - start.pitch() > 0.00005,
					"Support must not discard most of the remaining pitch motion");
			assertHullClear(wreck, terrain);
			for (int tick = 1; tick < 20; tick++) {
				physics.step(wreck, TICK_SECONDS, terrain, currents, config.battleArea(), tick);
				assertHullClear(wreck, terrain);
			}
			recording.stop();
			recording.dump(file);
		}
		var events = terrainEvents(file);
		assertEquals(20, events.size());
		assertTrue(events.stream().anyMatch(event -> event.getInt("sweptContacts") > 0),
				"The fixture must exercise actual support contacts rather than free motion");
		for (var event : events) {
			assertEquals(0, event.getDouble("remainingSeconds"), 1e-12,
					"Repeated seabed support must consume the whole motion budget");
			assertEquals(0, event.getInt("hpAtStart"));
			assertEquals(0, event.getInt("hpAtEnd"));
		}
	}

	@Test
	void supportOnAGridSeamConsumesTheWholeTick(@TempDir
	Path directory) throws IOException {
		// Tick 28537 of the other slow recording supports a point at x=2990 from the
		// left cell. An endpoint exactly on that seam must retain that established face.
		var config = MatchConfig.withDefaults(SEAM_SEED);
		var terrain = seamSeabed();
		var currents = seamCurrents();
		var wreck = new SubmarineEntity(VehicleConfig.submarine(), 7, null,
				new Vec3(2978.792056311075, 2877.8412964482354, -542.2204731481901), 0.46381189324292654, Color.BLUE,
				1000);
		wreck.setHp(0);
		wreck.setActualBallast(0);
		wreck.setPitch(0.09225256557701116);
		wreck.setSpeed(-0.6359277965237631);
		wreck.setSwaySpeed(0.711945653928405);
		wreck.setVerticalSpeed(0.29318029947753366);
		wreck.setYawRate(0.008929745861469142);
		wreck.setPitchRate(0.0013924945839308579);
		assertHullClear(wreck, terrain);
		var start = wreck.pose();
		var physics = new SubmarinePhysics(config);
		Path file = directory.resolve("wreck-grid-seam.jfr");
		try (var recording = new Recording()) {
			recording.enable(SimulationJfrEvents.TERRAIN_EVENT_NAME).withThreshold(Duration.ZERO).withoutStackTrace();
			recording.start();
			physics.step(wreck, TICK_SECONDS, terrain, currents, config.battleArea(), 28537);
			assertTrue(wreck.pose().position().subtract(start.position()).length() > 0.004,
					"Crossing a support seam must preserve the wreck's permitted drift");
			assertTrue(Math.abs(wreck.heading() - start.heading()) > 0.00005,
					"A seam must not discard most of the wreck's yaw motion");
			assertTrue(Math.abs(wreck.pitch() - start.pitch()) > 0.000005,
					"A seam must not discard most of the wreck's pitch motion");
			assertHullClear(wreck, terrain);
			for (int offset = 1; offset < 20; offset++) {
				physics.step(wreck, TICK_SECONDS, terrain, currents, config.battleArea(), 28537 + offset);
				assertHullClear(wreck, terrain);
			}
			recording.stop();
			recording.dump(file);
		}
		var events = terrainEvents(file);
		assertEquals(20, events.size());
		assertTrue(events.stream().anyMatch(event -> event.getInt("supportIterations") > 0),
				"The recorded fixture must exercise established seabed support");
		for (var event : events) {
			assertEquals(TICK_SECONDS, event.getDouble("tickDurationSeconds"), 1e-12);
			assertEquals(0, event.getDouble("remainingSeconds"), 1e-12,
					"Seam support must consume all 20 ms of each tick");
		}
	}

	@Test
	void seabedSupportStillSweepsForANewForebodyObstacle(@TempDir
	Path directory) throws IOException {
		var config = MatchConfig.withDefaults(17);
		var terrain = floorWithForebodyRidge();
		var currents = new CurrentField(List.of());
		var wreck = new SubmarineEntity(VehicleConfig.submarine(), 7, null, new Vec3(0, 0, -95), 0, Color.BLUE, 1000);
		wreck.setHp(0);
		wreck.setActualBallast(0);
		wreck.setSpeed(15);
		assertHullClear(wreck, terrain);
		assertTrue(HullGeometry.terrainPenetration(new Vec3(0, 1.5, -95), 0, 0, wreck.vehicleConfig(), terrain) > 0.4,
				"The unimpeded forebody must reach the ridge after the initial keel support");
		Path file = directory.resolve("wreck-new-obstacle.jfr");
		try (var recording = new Recording()) {
			recording.enable(SimulationJfrEvents.TERRAIN_EVENT_NAME).withThreshold(Duration.ZERO).withoutStackTrace();
			recording.start();
			new SubmarinePhysics(config).step(wreck, 0.1, terrain, currents, config.battleArea(), 0);
			recording.stop();
			recording.dump(file);
		}
		var events = terrainEvents(file);
		assertEquals(1, events.size());
		var event = events.getFirst();
		assertTrue(event.getInt("sweptContacts") >= 2,
				"After initial keel support, a newly reached ridge must receive its own swept impact response");
		assertEquals(0, event.getDouble("remainingSeconds"), 1e-12);
		assertTrue(wreck.speed() < 12,
				"The new ridge must impart a compression impulse rather than a speculative support correction");
		assertHullClear(wreck, terrain);
	}

	@Test
	void supportJustBeforeTheEndOfTheMotionDoesNotLaunchTheWreck(@TempDir
	Path directory) throws IOException {
		// A swept contact in the last microseconds of a tick leaves the support projection an
		// interval of nanoseconds. Its velocity target must stay bounded by the motion that
		// interval can hold; dividing a fixed contact margin by it launched wrecks at hundreds of
		// metres per second. The clearance band that produces such contacts is a few nanometres
		// wide, so it is bisected here rather than hard-coded.
		var config = MatchConfig.withDefaults(23);
		var terrain = rippledSlope();
		var currents = new CurrentField(List.of());
		double touching = touchingHeight(terrain);
		double contacting = 0, clear = 2;
		assertTrue(contactImpulse(config, terrain, currents, touching + contacting, 0) > 0.01,
				"The wreck must sweep into the slope when it starts on it");
		assertTrue(contactImpulse(config, terrain, currents, touching + clear, 1) < 0.01,
				"The wreck must clear the slope from two metres above it");
		for (int iteration = 0; iteration < 60; iteration++) {
			double middle = (contacting + clear) / 2;
			if (contactImpulse(config, terrain, currents, touching + middle, 2 + iteration) > 0.01) {
				contacting = middle;
			} else {
				clear = middle;
			}
		}
		Path file = directory.resolve("wreck-late-contact.jfr");
		try (var recording = new Recording()) {
			recording.enable(SimulationJfrEvents.TERRAIN_EVENT_NAME).withThreshold(Duration.ZERO).withoutStackTrace();
			recording.start();
			for (int step = 0; step < 64; step++) {
				double clearance = contacting - 2e-8 * step / 64;
				var wreck = driftingWreck(touching + clearance);
				var before = wreck.velocity().linear();
				new SubmarinePhysics(config).step(wreck, TICK_SECONDS, terrain, currents, config.battleArea(),
						1000 + step);
				double change = wreck.velocity().linear().subtract(before).length();
				assertTrue(change <= before.length(),
						"A frictionless seabed contact cannot change the wreck's velocity by more than its incoming "
								+ "speed, but changed it by " + change + " m/s at clearance " + clearance);
				assertHullClear(wreck, terrain);
			}
			recording.stop();
			recording.dump(file);
		}
		var events = terrainEvents(file);
		assertEquals(64, events.size());
		assertTrue(events.stream().anyMatch(event -> event.getDouble("lastContactFraction") > 0.9999
				&& event.getInt("supportIterations") > 0),
				"The scanned clearances must produce support after a swept contact at the very end of the motion");
	}

	private static double contactImpulse(
		MatchConfig config, TerrainMap terrain, CurrentField currents, double height, long tick) {
		var wreck = driftingWreck(height);
		var before = wreck.velocity().linear();
		new SubmarinePhysics(config).step(wreck, TICK_SECONDS, terrain, currents, config.battleArea(), tick);
		return wreck.velocity().linear().subtract(before).length();
	}

	private static SubmarineEntity driftingWreck(double height) {
		var wreck = new SubmarineEntity(VehicleConfig.submarine(), 7, null, new Vec3(3.3, 2.1, height), 0.7, Color.BLUE,
				1000);
		wreck.setHp(0);
		wreck.setActualBallast(0);
		wreck.setPitch(-0.02);
		wreck.setSpeed(0.3);
		wreck.setSwaySpeed(0.05);
		wreck.setVerticalSpeed(-0.001);
		wreck.setYawRate(0.002);
		return wreck;
	}

	/** Largest height at which the drifting wreck's hull does not penetrate the rippled slope. */
	private static double touchingHeight(TerrainMap terrain) {
		double inside = -200, clear = 0;
		for (int iteration = 0; iteration < 200; iteration++) {
			double middle = (inside + clear) / 2;
			if (HullGeometry.terrainPenetration(new Vec3(3.3, 2.1, middle), 0.7, -0.02, VehicleConfig.submarine(),
					terrain) > 0) {
				inside = middle;
			} else {
				clear = middle;
			}
		}
		return clear;
	}

	private static TerrainMap rippledSlope() {
		double[] elevations = new double[441];
		var roughness = new Random(7);
		for (int row = 0; row < 21; row++) {
			for (int column = 0; column < 21; column++) {
				elevations[row * 21 + column] = -100 + 1.5 * column + 0.4 * Math.sin(0.9 * row)
						+ 0.3 * roughness.nextDouble();
			}
		}
		return new TerrainMap(elevations, 21, 21, -100, -100, 10);
	}

	private static TerrainMap floorWithForebodyRidge() {
		int size = 61;
		double cell = 5;
		double origin = -150;
		double[] elevations = new double[size * size];
		for (int row = 0; row < size; row++) {
			double elevation = origin + row * cell == 40 ? -94 : -100;
			for (int column = 0; column < size; column++) {
				elevations[row * size + column] = elevation;
			}
		}
		return new TerrainMap(elevations, size, size, origin, origin, cell);
	}

	private static List<RecordedEvent> terrainEvents(Path file) throws IOException {
		return RecordingFile.readAllEvents(file).stream()
				.filter(event -> event.getEventType().getName().equals(SimulationJfrEvents.TERRAIN_EVENT_NAME))
				.toList();
	}

	private static void assertHullClear(SubmarineEntity wreck, TerrainMap terrain) {
		assertEquals(0, HullGeometry.terrainPenetration(wreck.pose().position(), wreck.heading(), wreck.pitch(),
				wreck.vehicleConfig(), terrain), 1e-8);
		assertEquals(0, wreck.hp());
	}

	private static SubmarineEntity supportedWreck() {
		// Derived from support on the Claude wreck's terrain in the slow seed12 recording.
		// The six-pass implementation leaves 18.145 ms of this 20 ms tick unspent.
		var wreck = new SubmarineEntity(VehicleConfig.submarine(), 7, null,
				new Vec3(2605.8028299773573, -3565.7972429858320, -161.67497615334625), 5.5443912361001475, Color.BLUE,
				1000);
		wreck.setHp(0);
		wreck.setActualBallast(0);
		wreck.setPitch(-0.19059660177357710);
		wreck.setSpeed(0.26173380376531247);
		wreck.setSwaySpeed(0.18705167341467600);
		wreck.setVerticalSpeed(-0.20633743065582197);
		wreck.setYawRate(0.0065585284980615010);
		wreck.setPitchRate(0.0095119793704821070);
		return wreck;
	}

	/**
	 * The 14 x 14 cells of 10 m seabed around the supported wreck, cropped from the generated world
	 * of seed {@value #SLOW_SEED} at columns 1105 and rows 486 onwards. Kept verbatim so that the
	 * fixture survives changes to the world generator.
	 */
	private static TerrainMap slowSeabed() {
		return seabed("""
			-213.7728795183794 -205.78422028808885 -200.0472004177347 -195.36050491482357 -193.6463420573923 -194.6449086790146 -195.7235884732969 -194.9881698646595 -193.98844688588383 -190.9295611269673 -187.2644957994821 -181.40927374770138 -172.02984218144513 -169.0246773177198
			-210.7640788441081 -204.0320277568993 -195.6821077435367 -188.6656807215159 -184.24889086985124 -185.30881834116136 -187.33780128539445 -188.70782761961544 -187.5686082783412 -186.49778648926323 -186.37184434255843 -183.4797172716089 -175.1336933512797 -164.47076232938505
			-206.50869914269558 -199.6797748557479 -188.85601602767161 -180.68053519968274 -175.74654019364135 -174.33008185003507 -178.64432005839694 -179.84648217004153 -181.0010910262727 -180.76615908133468 -181.19540523059905 -180.77359053924425 -174.5086332644798 -164.69689386708873
			-195.1241506428742 -187.93647533865249 -175.45048419383593 -168.38120879076087 -166.48568687838636 -167.54996816200224 -169.82138930160906 -173.1784413977041 -175.92287118568294 -176.80686462182032 -176.1567171067816 -175.13678798429112 -172.9686736593482 -167.81423867291878
			-180.20921883261678 -170.74227244235644 -160.9677347381839 -163.47824895136233 -164.4968125946083 -164.70466555645618 -163.5037608791804 -164.33003220721017 -168.07276545599152 -170.09701119805612 -171.34973486064274 -172.5340764724819 -171.94925172303323 -168.62689734838506
			-164.8604891729782 -157.89609670874896 -160.6709639579478 -162.84452529431294 -163.57566115119812 -163.24687004760438 -161.62006987475561 -159.43887161217594 -158.330771669342 -161.72994889499245 -164.28227614497192 -167.30660055336256 -168.690963451054 -167.46682135004602
			-158.87514366397983 -161.96685117773163 -161.081903192962 -160.84279352770136 -161.10425754299462 -160.06149999242766 -163.0187095129936 -163.24771808681413 -158.28165567260896 -152.26866995984375 -156.46358946388884 -161.13269260920538 -164.0915213856662 -164.53126053762784
			-165.10286434623177 -167.96656607744023 -167.06630902072789 -164.2936053170343 -163.63033482132755 -165.73363731727443 -168.03308378425083 -167.8448427225673 -163.3376747230231 -155.6371390263157 -153.28181111131926 -159.62881403098288 -164.68786967371494 -169.1499004160144
			-167.25147379778906 -169.9409434720308 -168.62885717412036 -167.50016218320775 -166.59366952681398 -168.41877606722974 -170.6106545732812 -170.1392922648003 -165.98368492194805 -159.67516696496142 -156.6465618014295 -161.58847692993677 -170.8598279255013 -176.65221860181282
			-171.00643552296566 -170.89940095085268 -171.3266210177433 -171.50063076724442 -170.70421196481485 -168.76720244790903 -169.91952505897774 -167.76804244292944 -163.2119061072962 -159.12641122705674 -159.24620040861572 -164.17318264304362 -174.34496553030124 -181.48036296495138
			-174.35288675948772 -175.42180321867286 -174.32170144792437 -174.6363635262038 -173.84934546901943 -171.90699584153248 -170.1045249842004 -168.37933713757542 -165.90953387667878 -162.27545366067102 -161.8071834406233 -165.70410528674583 -173.76501826505844 -180.6497302233572
			-180.5149155806937 -181.25113456515578 -178.58982689920208 -176.02120617414585 -174.65511968118074 -175.69339921004732 -173.73505742271652 -168.9259587107515 -166.94243485609283 -163.46752782673946 -164.81786725533632 -168.2998582498888 -171.396126162855 -174.58371529415777
			-181.49968050020576 -181.795129387794 -179.89948400011178 -177.00667590488416 -179.11617048119993 -182.8610525285553 -182.71320600491694 -178.75727573960035 -173.86982123137153 -170.94978323876828 -172.03190408451454 -172.4454978463173 -173.56646082634316 -175.14817696893553
			-181.66042358655753 -180.45000440193996 -181.08461931132155 -185.04935661278265 -187.69226023715362 -188.1762012162454 -186.85642196392752 -184.76787846329924 -182.44771196792436 -179.46766258435645 -178.99617943766992 -177.83674583536566 -176.810968578908 -179.09920505110946
				""", 2550, -3640);
	}

	private static CurrentField slowCurrents() {
		var bands = List.of(new CurrentBand(-224.0, -30.0, new Vec2(-0.24771800741437625, 0.16877232241399437)),
				new CurrentBand(-418.0, -224.0, new Vec2(-0.1254003857145454, -0.0667676272212101)),
				new CurrentBand(-612.0, -418.0, new Vec2(0.210232249225976, -0.03502170692573052)),
				new CurrentBand(-806.0, -612.0, new Vec2(-0.09526822519601388, -0.048382016809152134)),
				new CurrentBand(-1000.0, -806.0, new Vec2(0.41437503862840797, -0.31645840814892795)));
		return new CurrentField(bands);
	}

	/**
	 * The seabed around the grid-seam support of seed {@value #SEAM_SEED}, cropped from columns
	 * 1141 and rows 1131 onwards of its generated world.
	 */
	private static TerrainMap seamSeabed() {
		return seabed("""
			-493.3787841105063 -498.5919922181493 -504.1133171845744 -511.30435299452535 -519.9275219110718 -530.1834462654793 -539.3860953053218 -542.5377444904146 -542.5520111941335 -542.396333250894 -536.1239663225626 -525.0042231528053 -514.9586069934714 -515.0748896195701
			-505.7200229620804 -511.14003995440953 -515.4520695810903 -521.3793976027621 -529.365288128184 -535.855548885854 -541.9470466722987 -547.0424296504923 -548.9718700582622 -545.8050559589717 -537.3736781597029 -529.2830835979802 -520.4297538065786 -516.3542493514896
			-514.8065131513808 -519.5528895214036 -523.8715280011118 -533.2241404579664 -537.8606934613377 -543.0828425052441 -549.2865205065109 -553.1506776603361 -552.8925359682829 -547.4581727097761 -536.9935790967352 -532.48602415219 -525.8782501199557 -521.2421466584981
			-519.6100167304585 -524.5046736115467 -536.1200134430584 -543.3873463966515 -545.7442406427258 -546.1568216721868 -551.9783226010323 -555.1397620339573 -554.0606382183079 -548.0724053175809 -537.0449308324874 -530.0898402286341 -528.7396926214059 -522.7439909357167
			-521.9102412006293 -533.6335560078301 -543.6925538896828 -549.8359401109973 -551.7893840928773 -550.7220120943755 -550.2819716558774 -553.9791949035646 -553.2330637999303 -547.6965443026036 -537.6180028123578 -531.1593066337509 -528.2838560739467 -523.4735758022854
			-528.9255997897017 -539.1432190536467 -547.3697079809915 -552.5255929600635 -554.6784886749494 -554.6293485075068 -553.1288283860224 -549.850586330351 -551.0635111787255 -546.4849181332379 -541.6605304550293 -537.6172187690679 -532.4603415245169 -527.5474268301814
			-531.6156167435597 -540.0874517657966 -546.4558427442539 -550.7332622514425 -553.1303590253115 -554.6980886422411 -555.4947028683225 -549.9679914458551 -547.9099641056027 -545.2858028426801 -543.8822790019171 -540.1273515633302 -534.2694717814514 -527.5690696247942
			-528.0531150406473 -535.5961746310612 -540.7798632206279 -544.1153254519852 -546.6263345851423 -550.2416638641066 -553.7591449859995 -551.3095930848308 -544.8044608977125 -543.6958472944846 -542.1444994834978 -537.9002834305802 -530.7296256152213 -522.0017885164408
			-519.0538718201686 -526.5147387375315 -533.9439804884934 -538.3938510040216 -540.2925927198686 -543.2976913061173 -549.1198431177287 -549.913669312603 -543.3211479096695 -542.44050706908 -535.98819840542 -530.7907207319788 -527.3641607166153 -524.2862979819165
			-507.6617916333893 -521.7317024934732 -530.625628590943 -534.9451393001815 -540.1180070432423 -539.5805932546878 -543.4550574698869 -547.0730358099369 -543.7550911220111 -541.4531789401804 -533.1481249594176 -524.5933779762408 -525.2329214436957 -521.6531060534471
			-508.40352761682755 -518.4661526058098 -528.4785921130215 -537.2908052706425 -541.5961655173527 -540.5373422946003 -540.2817057330685 -542.6174647425332 -542.8015332479569 -539.2346563555602 -532.1659042298805 -525.1724122800246 -518.7551566590778 -514.9836480194319
			-508.11671062891435 -517.9583126555941 -528.1965760699886 -537.5429832721647 -541.783408993492 -540.7576376453284 -536.8195168248825 -539.6394395482612 -538.7010186857307 -535.058565996479 -528.4127308340557 -519.5081444078672 -510.91545280663047 -503.61581296924913
			-505.0296994044577 -519.1941888027964 -524.6001686967343 -534.7694762289177 -539.7489808710243 -539.1325702170057 -533.7991146135547 -529.7532180931935 -530.611870891436 -528.5494404689603 -520.3906067384946 -511.6579597545047 -500.45180319814415 -488.35070261591875
			-504.35843142622116 -517.6234654984023 -523.3658901913532 -528.6272358383803 -534.6936852671662 -534.6862263803931 -529.2701848851991 -522.3183397084944 -522.4153892928477 -519.7345260886484 -509.35997768207835 -502.8788584915459 -493.43247968051685 -481.2010521917133
				""", 2910, 2810);
	}

	private static CurrentField seamCurrents() {
		var bands = List.of(new CurrentBand(-272.5, -30.0, new Vec2(-0.27191727728835724, 0.29512541787617536)),
				new CurrentBand(-515.0, -272.5, new Vec2(0.10445692974808413, -0.4619443340489959)),
				new CurrentBand(-757.5, -515.0, new Vec2(-0.03045551901646135, 0.29844512546678187)),
				new CurrentBand(-1000.0, -757.5, new Vec2(-0.16903529072765797, -0.2391546724948578)));
		return new CurrentField(bands);
	}

	/** Parses rows of elevations, southernmost first, into a 14 x 14 map of 10 m cells. */
	private static TerrainMap seabed(String rows, double originX, double originY) {
		double[] elevations = Arrays.stream(rows.strip().split("\\s+")).mapToDouble(Double::parseDouble).toArray();
		assertEquals(14 * 14, elevations.length);
		return new TerrainMap(elevations, 14, 14, originX, originY, 10);
	}
}
