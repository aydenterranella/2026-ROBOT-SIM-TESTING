package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.constants.ShotConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Fires one stationary shot per row of the robot's shot table, with the flywheel and hood exactly
 * where the table says, and checks MapleSim counts each one in the blue hub. This is what ties the
 * sim's launch model to the real robot's tuning: if the table changes a lot, this catches it.
 */
class SimShotTest {
  // MapleSim's blue hub center (RebuiltHub.blueHubPose).
  private static final double HUB_X = 4.5974;
  private static final double HUB_Y = 4.034536;

  @BeforeAll
  static void setupHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  /**
   * Steps the arena with the robot clock paused and advanced 20 ms per loop. MapleSim times
   * projectiles with the WPILib clock, so without this a fast test loop barely moves the ball.
   */
  private static void runArena(double seconds) {
    SimHooks.pauseTiming();
    for (int t = 0; t < (int) (seconds / 0.02); t++) {
      SimHooks.stepTiming(0.02);
      SimulatedArena.getInstance().simulationPeriodic();
    }
    SimHooks.resumeTiming();
  }

  private static double fuelInBlueHub() {
    return SimulatedArena.getInstance().blueScoringBreakdown.getOrDefault("TotalFuelInHub", 0.0);
  }

  @Test
  void everyShotTableRowScoresWhenStationary() {
    for (int i = 0; i < ShotConstants.SHOOTER_SPEEDS.length; i++) {
      double distance = ShotConstants.SHOOTER_SPEEDS[i][0];
      double rps = ShotConstants.SHOOTER_SPEEDS[i][1];
      double hood = ShotConstants.HOOD_ANGLES[i][1];

      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
      SimulatedArena.getInstance().clearGamePieces();
      // Table distance is measured from the launcher, which sits behind the robot center. The
      // robot faces away from the hub (heading 180°) so its back, and the shooter, point at it.
      Pose2d robot =
          new Pose2d(
              HUB_X - distance - MechanismSimConstants.SHOT_EXIT_BEHIND_CENTER_M,
              HUB_Y,
              Rotation2d.k180deg);
      SimulatedArena.getInstance()
          .addGamePieceProjectile(SimShot.launch(robot, new ChassisSpeeds(), rps, hood));
      runArena(2.0);

      assertEquals(1.0, fuelInBlueHub(), 0.0, "shot from " + distance + " m should score");
    }
  }

  @Test
  void slowFlywheelFallsShort() {
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SimulatedArena.getInstance().clearGamePieces();
    double distance = 3.29; // table row: 74 rps, hood 0.060
    Pose2d robot =
        new Pose2d(
            HUB_X - distance - MechanismSimConstants.SHOT_EXIT_BEHIND_CENTER_M,
            HUB_Y,
            Rotation2d.k180deg);
    // Flywheel still spinning up (20% under speed): the ball should not make it.
    SimulatedArena.getInstance()
        .addGamePieceProjectile(SimShot.launch(robot, new ChassisSpeeds(), 74 * 0.8, 0.060));
    runArena(2.0);
    assertEquals(0.0, fuelInBlueHub(), 0.0, "an under-speed shot should miss");
  }
}
