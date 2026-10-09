package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Fuel inside the robot: hopper → spindexer → feeder → flywheel. */
class FuelPathSimTest {
  // Typical simulated speeds while firing (rotor rps): spindexer ~80, feeder ~-110 (feeding).
  private static final double SPINDEXER = 80.0;
  private static final double FEEDER = -110.0;

  @BeforeAll
  static void setupHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void loadedHopperFiresAtAboutEightPerSecond() {
    FuelPathSim fuel = new FuelPathSim();
    fuel.setCount(20);
    int shots = 0;
    int loops = 0;
    double firstShotSecs = -1;
    int shotsAtOneSecond = 0;
    while (fuel.getCount() > 0 && loops < 500) {
      shots += fuel.update(SPINDEXER, FEEDER);
      loops++;
      if (firstShotSecs < 0 && shots > 0) firstShotSecs = loops * 0.02;
      if (loops == 50 + (int) (firstShotSecs / 0.02)) shotsAtOneSecond = shots;
    }
    assertEquals(20, shots, "every ball should come out the top");
    // The first ball has to ride to the feeder entrance and climb the feeder before it fires.
    assertTrue(firstShotSecs > 0.15 && firstShotSecs < 0.8, "first shot after " + firstShotSecs + " s");
    // Over the first second of firing: about 8, the team's real rate.
    assertTrue(shotsAtOneSecond >= 6 && shotsAtOneSecond <= 10,
        "fired " + shotsAtOneSecond + " in the first second of firing");
  }

  @Test
  void nothingFiresUnlessBothSpindexerAndFeederRun() {
    FuelPathSim fuel = new FuelPathSim();
    fuel.setCount(10);
    int shots = 0;
    for (int i = 0; i < 100; i++) shots += fuel.update(0.0, 0.0); // robot not ready: both stopped
    for (int i = 0; i < 100; i++) shots += fuel.update(SPINDEXER, 0.0); // feeder stopped
    assertEquals(0, shots, "no shots while the feeder is stopped");
    // Fire briefly, then stop both (trigger released / not ready): shots stop right away, and the
    // balls left in the feeder and hopper stay put.
    for (int i = 0; i < 30; i++) shots += fuel.update(SPINDEXER, FEEDER);
    int whenStopped = shots;
    for (int i = 0; i < 100; i++) shots += fuel.update(0.0, 0.0);
    assertEquals(whenStopped, shots, "no shots after the feeder stops");
    assertTrue(fuel.getCount() > 0, "the rest should still be in the robot");
    assertEquals(10, fuel.getCount() + shots, "no balls lost");
  }

  @Test
  void pickedUpBallLandsInTheHopperThenGetsShot() {
    SimHooks.pauseTiming();
    try {
      FuelPathSim fuel = new FuelPathSim();
      fuel.addFromIntake(new Translation3d(0.55, 0.0, 0.075));
      assertEquals(1, fuel.getCount(), "counts as held while flying in");
      assertEquals(1, fuel.getPoses(new Pose3d()).size(), "drawn while flying in");
      // Mid-flight it should be above the floor, heading up over the roller.
      SimHooks.stepTiming(MechanismSimConstants.FEED_ANIMATION_SECS / 2);
      assertTrue(fuel.getPoses(new Pose3d()).get(0).getZ() > 0.2, "arcs up over the roller");

      SimHooks.stepTiming(MechanismSimConstants.FEED_ANIMATION_SECS);
      int shots = 0;
      for (int i = 0; i < 100 && shots == 0; i++) {
        shots += fuel.update(SPINDEXER, FEEDER);
        SimHooks.stepTiming(0.02);
      }
      assertEquals(1, shots, "lands, rides the spindexer, climbs the feeder, and shoots");
      assertEquals(0, fuel.getCount());
    } finally {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void everyBallIsDrawnWhileFiring() {
    FuelPathSim fuel = new FuelPathSim();
    fuel.setCount(MechanismSimConstants.HOPPER_CAPACITY);
    assertEquals(MechanismSimConstants.HOPPER_CAPACITY, fuel.getPoses(new Pose3d()).size());
    for (int i = 0; i < 40; i++) {
      fuel.update(SPINDEXER, FEEDER);
      assertEquals(fuel.getCount(), fuel.getPoses(new Pose3d()).size(),
          "heap + spindexer + feeder balls should all be drawn");
    }
  }
}
