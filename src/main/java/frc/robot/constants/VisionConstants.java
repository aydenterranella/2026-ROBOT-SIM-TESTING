package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final String CAMERA_NAME = "limelight";
  public static final double CAMERA_FOV_DEGREES = 60.0;
  public static final int IMAGE_WIDTH = 320;
  public static final int IMAGE_HEIGHT = 240;
  public static final double TARGET_AREA_THRESHOLD = 1.0;
  public static final double MAX_TARGET_DISTANCE_METERS = 5.0;

  public static final String LEFT_TABLE = "limelight-left";
  public static final String RIGHT_TABLE = "limelight-right";
  public static final String SHOOTER_TABLE = "limelight-shooter";

  // Robot center (on the floor) → camera lens, WPILib robot frame (+X intake side, +Y left, +Z up).
  // Measured from the Limelight field-of-view pyramids in the CAD Main Assembly: the pyramid tip is
  // the lens and its axis is where the camera points. Pitch is negative because the cameras tilt UP.
  // All three are Limelight 4s.

  /** Rear, above the shooter: 16.5" up, facing straight back, tilted 10° up. */
  public static final Transform3d ROBOT_TO_SHOOTER_CAMERA =
      new Transform3d(
          -0.3467, 0.0, 0.4194, new Rotation3d(0.0, Math.toRadians(-10.0), Math.toRadians(180.0)));

  /** Left side: 9" up, facing straight out to the left, tilted 20° up. */
  public static final Transform3d ROBOT_TO_LEFT_CAMERA =
      new Transform3d(
          -0.2666, 0.2066, 0.2289, new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(90.0)));

  /** Right side: mirror of the left camera. */
  public static final Transform3d ROBOT_TO_RIGHT_CAMERA =
      new Transform3d(
          -0.2666, -0.2066, 0.2289, new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(-90.0)));

  // Limelight 4 sensor: 1280x800, 82° x 56.2° field of view (91.1° diagonal).
  public static final int LL4_RESOLUTION_WIDTH = 1280;
  public static final int LL4_RESOLUTION_HEIGHT = 800;
  public static final double LL4_HORIZONTAL_FOV_DEG = 82.0;
  public static final double LL4_VERTICAL_FOV_DEG = 56.2;
}
