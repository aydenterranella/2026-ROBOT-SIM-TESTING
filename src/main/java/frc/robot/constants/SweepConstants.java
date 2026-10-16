package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SweepConstants {
  // neutral zone center (derived from FieldConstants neutral boundaries)
  public static final double NEU_CENTER_X = (FieldConstants.NEU_TOP_LEFT.getX() + FieldConstants.NEU_TOP_RIGHT.getX()) / 2.0;
  public static final double NEU_CENTER_Y = (FieldConstants.NEU_TOP_LEFT.getY() + FieldConstants.NEU_BOTTOM_LEFT.getY()) / 2.0;

  // how far from center to push toward each edge
  public static final double SWEEP_MARGIN = 0.5;

  // drive speed multiplier while sweeping (0.0 - 1.0)
  public static final double SWEEP_DRIVE_SPEED = 0.6;

  // target poses for each sweep direction
  // TOP: push balls toward the top wall of the neutral zone
  public static final Pose2d TOP_TARGET = new Pose2d(
    NEU_CENTER_X,
    FieldConstants.NEU_TOP_LEFT.getY() - SWEEP_MARGIN,
    Rotation2d.fromDegrees(90.0)
  );

  // BOTTOM: push balls toward the bottom wall of the neutral zone
  public static final Pose2d BOTTOM_TARGET = new Pose2d(
    NEU_CENTER_X,
    FieldConstants.NEU_BOTTOM_LEFT.getY() + SWEEP_MARGIN,
    Rotation2d.fromDegrees(-90.0)
  );

  // LEFT: push balls toward the blue/left side of the neutral zone
  public static final Pose2d LEFT_TARGET = new Pose2d(
    FieldConstants.NEU_TOP_LEFT.getX() + SWEEP_MARGIN,
    NEU_CENTER_Y,
    Rotation2d.fromDegrees(180.0)
  );

  // RIGHT: push balls toward the red/right side of the neutral zone
  public static final Pose2d RIGHT_TARGET = new Pose2d(
    FieldConstants.NEU_TOP_RIGHT.getX() - SWEEP_MARGIN,
    NEU_CENTER_Y,
    Rotation2d.fromDegrees(0.0)
  );
}
