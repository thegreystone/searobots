package se.hirt.searobots.engine;

import se.hirt.searobots.api.*;
import java.util.List;

public class HeadlessRun {
    public static void main(String[] args) {
        long seed = args.length > 0 ? Long.parseLong(args[0]) : -7475901583965440089L;
        var world = new WorldGenerator().generate(MatchConfig.withDefaults(seed));
        var sim = new SimulationLoop();

        List<SubmarineController> controllers = List.of(new DefaultAttackSub(), new SubmarineDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.submarine());

        sim.run(world, controllers, configs, new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> subs) {
                for (var s : subs) {
                    double yawDeg = Math.toDegrees(s.pose().heading());
                    double pitchDeg = Math.toDegrees(s.pose().pitch());
                    // Check for crazy values
                    if (Math.abs(s.speed()) > 30 || Math.abs(s.pose().position().x()) > 20000
                            || Math.abs(s.pose().position().y()) > 20000
                            || Math.abs(pitchDeg) > 60) {
                        System.out.printf("ALERT tick=%d id=%d name=%s x=%.0f y=%.0f z=%.1f hdg=%.1f pitch=%.1f spd=%.1f rudder=%.2f planes=%.2f throttle=%.2f status=%s%n",
                                tick, s.id(), s.name(), s.pose().position().x(), s.pose().position().y(),
                                s.pose().position().z(), yawDeg, pitchDeg, s.speed(),
                                s.rudder(), s.sternPlanes(), s.throttle(), s.status());
                        sim.stop();
                        return;
                    }
                    if (tick % 500 == 0 && s.id() == 0) {
                        System.out.printf("tick=%d id=%d x=%.0f y=%.0f z=%.1f hdg=%.1f pitch=%.1f spd=%.1f rudder=%.2f planes=%.2f status=%s%n",
                                tick, s.id(), s.pose().position().x(), s.pose().position().y(),
                                s.pose().position().z(), yawDeg, pitchDeg, s.speed(),
                                s.rudder(), s.sternPlanes(), s.status());
                    }
                }
            }

            @Override
            public void onMatchEnd() {
                System.out.println("Match ended.");
            }
        });
    }
}
