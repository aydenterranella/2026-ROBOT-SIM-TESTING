// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToPointCommand extends Command {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public double MAX_DRIVE_TO_POINT_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // * 0.75;
  public double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.5)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE = RotationsPerSecond.of(1)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double STATIC_FRICTION_CONSTANT = 0.085; // Adjust this value based on your robot's characteristics
  public double SKEW_COMPENSATION = 0. - 0.03; // Adjust this value based on your robot's characteristics

  private final Swerve drivetrain;
  private Translation2d targetPoint;
  private double rotationTolerance;
  private Rotation2d targetRotationToPoint;

  /** Creates a new DriveCommand. */
  public TurnToPointCommand(
      Swerve drivetrain,
      Translation2d point,
      double tolerance) {
    this.drivetrain = drivetrain;
    this.targetPoint = point;
    this.rotationTolerance = tolerance;

    addRequirements(drivetrain);
  }

  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  @Override
  public void initialize() {
    Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
    double deltaX = targetPoint.getX() - robotPosition.getX();
    double deltaY = targetPoint.getY() - robotPosition.getY();
    double targetAngleFieldRelative = Math.atan2(deltaY, deltaX);
    // Blue: add 180° so rear faces target. Red: CTRE operator perspective adds 180° for us.
    double offset = isBlueAlliance() ? Math.PI : 0.0;
    targetRotationToPoint = Rotation2d.fromRadians(targetAngleFieldRelative + offset);
  }

  @Override
  public void execute() {
    drivetrain.drive(
        0.0,
        0.0,
        targetRotationToPoint.getDegrees(),
        Swerve.DriveMode.ROTATION_LOCK);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return Math
        .abs(drivetrain.getState().Pose.getRotation().minus(targetRotationToPoint).getDegrees()) < rotationTolerance;
  }
}