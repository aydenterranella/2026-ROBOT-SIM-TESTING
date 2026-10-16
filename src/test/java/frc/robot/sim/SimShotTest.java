package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants;
import frc.robot.utils.ShootingUtils;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Fires one stationary shot per row of the robot's shot table, with the flywheel and hood exactly
 * where the robot code commands them, and checks MapleSim counts each one in the blue hub. This is
 * what ties the sim's launch model to the real robot's tuning: if the table changes a lot, this
 * catches it.
 */
class SimShotTest {
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

  /** Shots come in on a diagonal so the 4.9 m row still fits between the hub and the wall. */
  private static final Rotation2d APPROACH = Rotation2d.fromDegrees(40);

  /**
   * Robot {@code distance} meters (center to hub, as the robot code measures it) from the blue hub,
   * facing away from it so its back, and the shooter, point at the hub.
   */
  private static Pose2d facingHub(double distance) {
    var hub = Constants.Shooting.BLUE_HUB;
    var toRobot = new Translation2d(distance, APPROACH.plus(Rotation2d.k180deg));
    return new Pose2d(hub.plus(toRobot), APPROACH.plus(Rotation2d.k180deg));
  }

  private static double fireStationary(Pose2d robot, double rps, double hood) {
    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    SimulatedArena.getInstance().clearGamePieces();
    SimulatedArena.getInstance()
        .addGamePieceProjectile(SimShot.launch(robot, new ChassisSpeeds(), rps, hood));
    runArena(2.0);
    return fuelInBlueHub();
  }

  @Test
  void everyShotTableRowScoresWhenStationary() {
    for (double[] row : Constants.Shooting.SHOOTER_SPEEDS) {
      double distance = row[0];
      Pose2d robot = facingHub(distance);
      var shot = ShootingUtils.calculate(robot, Alliance.Blue, Double.NaN, Double.NaN, true);
      assertTrue(shot.valid(), "table row " + distance + " m should be a valid shot");

      assertEquals(1.0, fireStationary(robot, shot.shooterRps(), shot.hoodRotations()), 0.0,
          "shot from " + distance + " m (" + shot.shooterRps() + " rps, hood "
              + shot.hoodRotations() + ") should score");
    }
  }

  @Test
  void slowFlywheelFallsShort() {
    Pose2d robot = facingHub(3.0);
    var shot = ShootingUtils.calculate(robot, Alliance.Blue, Double.NaN, Double.NaN, true);
    // Flywheel still spinning up (20% under speed): the ball should not make it.
    assertEquals(0.0, fireStationary(robot, shot.shooterRps() * 0.8, shot.hoodRotations()), 0.0,
        "an under-speed shot should miss");
  }
}
