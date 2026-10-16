package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

/**
 * Drives {@link RobotBumpSim} straight at the blue bump with a stand-in for MapleSim (constant
 * commanded X speed off the ramp, bump sim owns X on the ramp, same as Swerve.updateBumpSim()).
 * Coasting, crest speed is sqrt(2·g·h) ≈ 1.8 m/s, so 1 m/s should slide back and 2.5 m/s should
 * cross; a robot that keeps driving climbs at 1 m/s.
 */
class RobotBumpSimTest {
  private static final double DT = 0.02;
  private static final double BUMP_ASCENT_START_X = 3.96;
  private static final double BUMP_PEAK_X = 4.61;
  private static final double BUMP_END_X = 5.18;

  /** Same module layout as Swerve.initMapleSim(): FL, FR, BL, BR at ±11.5" × ±10.5". */
  private static final Translation2d[] MODULES = {
    new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(10.5)),
    new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(-10.5)),
    new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(10.5)),
    new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-10.5)),
  };

  private record Result(double maxX, double finalX, double maxZ, double minPitch, boolean everOnRamp) {}

  private static Result drive(double commandedVx, double seconds) {
    return drive(commandedVx, seconds, false);
  }

  /**
   * @param driven true: the stand-in for MapleSim keeps pushing toward the commanded speed on the
   *     ramp (up to 3 m/s² per loop), as the wheels do; false: it coasts at the bump sim's speed
   */
  private static Result drive(double commandedVx, double seconds, boolean driven) {
    RobotBumpSim sim = new RobotBumpSim(MODULES);
    // Y = 2.5 m is inside the lower blue bump's Y range.
    Pose2d pose = new Pose2d(3.0, 2.5, Rotation2d.kZero);
    double vx = commandedVx;
    double maxX = pose.getX(), maxZ = 0, minPitch = 0;
    boolean everOnRamp = false;

    for (int i = 0; i < (int) (seconds / DT); i++) {
      pose = new Pose2d(pose.getX() + vx * DT, pose.getY(), pose.getRotation());
      Pose3d pose3d = sim.update(pose, new ChassisSpeeds(vx, 0, 0), 5);
      if (sim.isOnRamp()) {
        everOnRamp = true;
        pose = sim.getSimWorldPose(pose);
        vx = sim.getSimXVelocity();
        if (driven) vx += Math.max(-3 * DT, Math.min(3 * DT, commandedVx - vx));
      } else {
        vx = commandedVx;
      }
      maxX = Math.max(maxX, pose.getX());
      maxZ = Math.max(maxZ, pose3d.getZ());
      minPitch = Math.min(minPitch, pose3d.getRotation().getY());
    }
    return new Result(maxX, pose.getX(), maxZ, minPitch, everOnRamp);
  }

  @Test
  void slowRobotSlidesBackOffBump() {
    Result r = drive(1.0, 3.0);
    assertTrue(r.everOnRamp(), "robot never touched the bump");
    assertTrue(r.maxX() < BUMP_PEAK_X, "robot crested the bump at 1 m/s: maxX=" + r.maxX());
  }

  @Test
  void fastRobotCrossesBump() {
    Result r = drive(2.5, 3.0);
    assertTrue(r.everOnRamp(), "robot never touched the bump");
    assertTrue(r.finalX() > BUMP_END_X + 0.5, "robot failed to cross at 2.5 m/s: finalX=" + r.finalX());
    assertTrue(r.maxZ() > 0.1, "robot never lifted: maxZ=" + r.maxZ());
    // Nose-up on the ascent is negative pitch in WPILib's convention.
    assertTrue(r.minPitch() < Math.toRadians(-5), "robot never pitched up: minPitch=" + r.minPitch());
  }

  @Test
  void drivenRobotClimbsBumpBelowCoastingCrestSpeed() {
    Result r = drive(1.0, 4.0, true);
    assertTrue(r.everOnRamp(), "robot never touched the bump");
    assertTrue(r.finalX() > BUMP_END_X + 0.5, "driven robot failed to climb at 1 m/s: finalX=" + r.finalX());
    assertTrue(r.maxZ() > 0.1, "robot never lifted: maxZ=" + r.maxZ());
  }

  @Test
  void resetClearsRampState() {
    RobotBumpSim sim = new RobotBumpSim(MODULES);
    // Front modules just onto the foot of the ramp (teleporting mid-ramp puts them under it).
    Pose2d onBump =
        new Pose2d(BUMP_ASCENT_START_X - MODULES[0].getX() + 0.03, 2.5, Rotation2d.kZero);
    for (int i = 0; i < 5; i++) sim.update(onBump, new ChassisSpeeds(1.0, 0, 0), 5);
    assertTrue(sim.isOnRamp());

    sim.reset();
    assertFalse(sim.isOnRamp());
    Pose3d flat = sim.update(new Pose2d(1.0, 1.0, Rotation2d.kZero), new ChassisSpeeds(), 5);
    assertEquals(1.0, flat.getX(), 1e-9);
    assertEquals(0.0, flat.getZ(), 1e-9);
  }
}
