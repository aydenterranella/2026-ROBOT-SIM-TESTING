package frc.robot.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.sim.MechanismSimConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Builds the MapleSim projectile for one shot from where the flywheel and hood actually are. See
 * the "Shots" section of {@link MechanismSimConstants} for how the launch model was calibrated
 * against the robot's shot table.
 */
public final class SimShot {
  private SimShot() {}

  /**
   * @param robotPose robot pose on the field
   * @param fieldSpeeds robot velocity, field-relative (the ball inherits it)
   * @param flywheelRps simulated flywheel rotor speed
   * @param hoodRot simulated hood position, code units (rotations)
   */
  public static RebuiltFuelOnFly launch(
      Pose2d robotPose, ChassisSpeeds fieldSpeeds, double flywheelRps, double hoodRot) {
    double speedMps = flywheelRps * SHOT_SPEED_MPS_PER_FLYWHEEL_RPS;
    double launchAngleDeg = SHOT_ANGLE_INTERCEPT_DEG + SHOT_ANGLE_DEG_PER_HOOD_ROT * hoodRot;
    // The shooter points out the back of the robot. RebuiltFuelOnFly rotates
    // shooterPositionOnRobot by shooterFacing (not the chassis heading), so (+x, 0) with a rear
    // facing puts the exit point x meters BEHIND the chassis center.
    return new RebuiltFuelOnFly(
        robotPose.getTranslation(),
        new Translation2d(SHOT_EXIT_BEHIND_CENTER_M, 0),
        fieldSpeeds,
        robotPose.getRotation().plus(Rotation2d.k180deg),
        Meters.of(SHOT_EXIT_HEIGHT_M),
        MetersPerSecond.of(speedMps),
        Degrees.of(launchAngleDeg));
  }
}
