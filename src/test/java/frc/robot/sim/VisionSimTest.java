package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LimelightHelpers;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

/**
 * Runs the real {@link Robot} with the simulated Limelights and checks that the vision code sees
 * tags, gets a MegaTag2 pose near the truth, and pulls a drifted pose estimate back.
 */
class VisionSimTest {
  /** Lens direction in the robot frame: should match where the CAD pyramids point. */
  @Test
  void camerasPointWhereCadSays() {
    Translation3d shooter = forward(VisionConstants.ROBOT_TO_SHOOTER_CAMERA);
    assertEquals(-Math.cos(Math.toRadians(10)), shooter.getX(), 1e-6, "shooter cam faces back");
    assertEquals(Math.sin(Math.toRadians(10)), shooter.getZ(), 1e-6, "shooter cam tilts up 10°");

    Translation3d left = forward(VisionConstants.ROBOT_TO_LEFT_CAMERA);
    assertEquals(Math.cos(Math.toRadians(20)), left.getY(), 1e-6, "left cam faces left");
    assertEquals(Math.sin(Math.toRadians(20)), left.getZ(), 1e-6, "left cam tilts up 20°");

    Translation3d right = forward(VisionConstants.ROBOT_TO_RIGHT_CAMERA);
    assertEquals(-Math.cos(Math.toRadians(20)), right.getY(), 1e-6, "right cam faces right");
    assertEquals(Math.sin(Math.toRadians(20)), right.getZ(), 1e-6, "right cam tilts up 20°");
  }

  @Test
  void visionSeesTagsAndCorrectsDrift() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    var visibleTags =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Sim/Vision/Shooter/VisibleTags", Pose3d.struct)
            .subscribe(new Pose3d[0]);

    Robot robot = new Robot();
    robot.robotInit();
    robot.simulationInit();
    Field container = Robot.class.getDeclaredField("m_robotContainer");
    container.setAccessible(true);
    Swerve drive = ((RobotContainer) container.get(robot)).getStateMachine().drivetrain;
    run(robot, 0.2);

    // Back (shooter side) toward the blue hub, 2.6 m away: the shooter camera sees the hub tags.
    Pose2d truth = new Pose2d(2.0, 4.03, Rotation2d.k180deg);
    drive.resetPose(truth);
    run(robot, 0.5);
    assertTrue(LimelightHelpers.getTV(VisionConstants.SHOOTER_TABLE),
        "shooter Limelight should have a target");
    assertTrue(visibleTags.get().length >= 1, "shooter camera should report visible tags");
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.SHOOTER_TABLE);
    assertTrue(mt2 != null && mt2.tagCount > 0, "should get a MegaTag2 estimate");
    assertEquals(0.0, mt2.pose.getTranslation().getDistance(truth.getTranslation()), 0.15,
        "MegaTag2 pose should be near the true pose");

    // Enable (match started, so vision only nudges the estimate), knock the estimate 0.5 m off
    // without moving the robot, and let vision pull it back.
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();
    run(robot, 0.2);
    drive.resetEstimatedPose(
        new Pose2d(truth.getX() + 0.5, truth.getY(), truth.getRotation()));
    run(robot, 4.0);
    double error =
        drive.getState().Pose.getTranslation().getDistance(drive.getSimWorldPose().getTranslation());
    assertTrue(error < 0.2, "vision should pull the estimate back to the truth, still off " + error);
    robot.close();
  }

  private static Translation3d forward(Transform3d robotToCamera) {
    return new Translation3d(1, 0, 0).rotateBy(robotToCamera.getRotation());
  }

  private static void run(Robot robot, double seconds) throws InterruptedException {
    for (int i = 0; i < (int) (seconds / 0.02); i++) {
      Unmanaged.feedEnable(100);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      robot.simulationPeriodic();
      Thread.sleep(20);
    }
  }
}
