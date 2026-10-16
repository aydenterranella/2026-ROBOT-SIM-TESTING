package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

// Subsystem for vision (Limelight) — robot pose estimation via AprilTags.
public class Vision extends SubsystemBase {

  public Vision() {}

  public static NetworkTable getTable(String name) {
    return NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void periodic() {}

  // Returns the name of the Limelight with the best (largest) tag area,
  // checking shooter, left, and right cameras in priority order.
  public String getBestLimelight() {
    LimelightHelpers.PoseEstimate shooterMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.SHOOTER_TABLE);
    LimelightHelpers.PoseEstimate leftMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.LEFT_TABLE);
    LimelightHelpers.PoseEstimate rightMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.RIGHT_TABLE);

    SmartDashboard.putNumber("shooterLLArea", shooterMeasurement != null ? shooterMeasurement.avgTagArea : 0);
    SmartDashboard.putNumber("leftLLArea",    leftMeasurement    != null ? leftMeasurement.avgTagArea    : 0);
    SmartDashboard.putNumber("rightLLArea",   rightMeasurement   != null ? rightMeasurement.avgTagArea   : 0);

    String best = "";
    double bestArea = 0;

    if (shooterMeasurement != null && shooterMeasurement.avgTagArea > bestArea) {
      best     = Constants.Vision.SHOOTER_TABLE;
      bestArea = shooterMeasurement.avgTagArea;
    }
    if (leftMeasurement != null && leftMeasurement.avgTagArea > bestArea) {
      best     = Constants.Vision.LEFT_TABLE;
      bestArea = leftMeasurement.avgTagArea;
    }
    if (rightMeasurement != null && rightMeasurement.avgTagArea > bestArea) {
      best = Constants.Vision.RIGHT_TABLE;
    }

    PowerRobotContainer.setData("bestLimelight", best);
    return best;
  }
}
