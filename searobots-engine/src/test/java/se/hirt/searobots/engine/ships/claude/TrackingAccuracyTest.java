package se.hirt.searobots.engine.ships.claude;
import se.hirt.searobots.engine.*;
import se.hirt.searobots.engine.ships.*;

import org.junit.jupiter.api.Test;
import se.hirt.searobots.api.*;
import java.util.*;

class TrackingAccuracyTest {

    @Test
    void measureEstimationAccuracy() {
        long seed = 42;
        var config = MatchConfig.withDefaults(seed);
        var world = new WorldGenerator().generate(config);
        var sim = new SimulationLoop();
        sim.setSpeedMultiplier(1_000_000);

        List<SubmarineController> controllers = List.of(new ClaudeAttackSub(), new TargetDrone());
        List<VehicleConfig> configs = List.of(VehicleConfig.submarine(), VehicleConfig.surfaceShip());

        System.out.printf("%-8s %-8s %-8s %-8s %-8s %-8s %-10s%n",
                "Time", "ActDist", "EstDist", "PosErr", "Hdg", "Spd", "Status");
        System.out.println("-".repeat(70));

        var listener = new SimulationListener() {
            @Override
            public void onTick(long tick, List<SubmarineSnapshot> submarines) {
                if (tick % 250 != 0 || submarines.size() < 2) return;
                var s0 = submarines.get(0);
                var s1 = submarines.get(1);
                var p0 = s0.pose().position();
                var p1 = s1.pose().position();
                double actualDist = Math.sqrt(Math.pow(p0.x()-p1.x(),2) + Math.pow(p0.y()-p1.y(),2));

                var contacts = s0.contactEstimates();
                if (contacts == null || contacts.isEmpty()) return;
                var ce = contacts.getFirst();

                double posErr = Math.sqrt(Math.pow(ce.x()-p1.x(),2) + Math.pow(ce.y()-p1.y(),2));
                double estDist = Math.sqrt(Math.pow(ce.x()-p0.x(),2) + Math.pow(ce.y()-p0.y(),2));

                String hdg = Double.isNaN(ce.estimatedHeading()) ? "n/a" :
                        String.format("%.0f", Math.toDegrees(ce.estimatedHeading()));
                String spd = ce.estimatedSpeed() < 0 ? "n/a" :
                        String.format("%.1f", ce.estimatedSpeed());
                String status = s0.status() != null ?
                        s0.status().substring(0, Math.min(10, s0.status().length())) : "";

                if (actualDist < 3000) {
                    System.out.printf("%-8.0f %-8.0f %-8.0f %-8.0f %-8s %-8s %-10s%n",
                            tick/50.0, actualDist, estDist, posErr, hdg, spd, status);
                }
            }
            @Override public void onMatchEnd() {}
        };

        var thread = new Thread(() -> sim.run(world, controllers, configs, listener));
        thread.start();
        try { thread.join(60_000); } catch (InterruptedException e) {}
        sim.stop();
        try { thread.join(5000); } catch (InterruptedException e) {}
    }
}
