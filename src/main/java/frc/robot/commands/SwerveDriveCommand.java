// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.DriveUtil;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.GameZone;


import frc.robot.constants.OIConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.DriveMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveCommand extends Command {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public double MAX_DRIVE_TO_POINT_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // * 0.75;
  public double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.5)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE = RotationsPerSecond.of(1)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double STATIC_FRICTION_CONSTANT = 0.085; // Adjust this value based on your robot's characteristics
  public double SKEW_COMPENSATION = 0. - 0.03; // Adjust this value based on your robot's characteristics

  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final boolean autoDrive;
  private final PIDController driveToPointController;
  private Pose2d requestedPose;
  private double poseTolerance;
  /**
   * If set, scales MAX_DRIVE_TO_POINT_SPEED for drive-to-point (0.0 to 1.0 = 0%
   * to 100%).
   */
  private final Double driveToPointSpeedMultiplier;
  private final boolean disableRotationLock;
  /** If set, used for SOTM heading lock during shooting. */
  private final StateMachine stateMachine;

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = null;
    this.poseTolerance = -1;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  /**
   * Teleop drive with SOTM heading lock during shooting.
   * Pass the StateMachine so the command can read the SOTM aim angle.
   */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      StateMachine stateMachine) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = null;
    this.poseTolerance = -1;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = stateMachine;

    addRequirements(drivetrain);
  }

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = -1.0;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  /**
   * Creates a new DriveCommand with requested pose, pose tolerance, and
   * drive-to-point speed as a percent of default.
   *
   * @param driveToPointSpeedMultiplier scale for max drive-to-point speed (0.0 to
   *                                    1.0 = 0% to 100% of
   *                                    {@link #MAX_DRIVE_TO_POINT_SPEED})
   */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance,
      double driveToPointSpeedMultiplier) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointSpeedMultiplier = driveToPointSpeedMultiplier;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance,
      double driveToPointSpeedMultiplier,
      boolean disableRotationLock) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointSpeedMultiplier = driveToPointSpeedMultiplier;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = disableRotationLock;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    final double speedCoefficient = FieldUtils.getZone(currentPose) == GameZone.INTERCHANGE
        ? 1.0 //? OIConstants.INTERCHANGE_SPEED_COEFFICIENT
        : 1.0;
    Pose2d targetPose = new Pose2d();
    targetPose = requestedPose;
    // Aim off the WANTED state: the trigger binding sets it this loop, but the state machine only
    // switches currentState on its next periodic, so checking currentState left a gap here.
    RobotState aimState = stateMachine != null ? stateMachine.getWantedState() : null;
    boolean shootHeld = controller.rightTrigger(0.5).getAsBoolean() && stateMachine != null;
    if (!shootHeld) wheelsParked = false;
    if (shootHeld && aimState == RobotState.PASSING) {
      boolean isInverted = SmartDashboard.getBoolean("driveInverted", false);
      double inversionMultiplier = isInverted ? -1.0 : 1.0;
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(
          controller, drivetrain, MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      double xSpeed = speeds.vxMetersPerSecond * OIConstants.MAX_SPEED_COEFFICIENT * inversionMultiplier * SLOW_DRIVE_SPEED;
      double ySpeed = speeds.vyMetersPerSecond * OIConstants.MAX_SPEED_COEFFICIENT * inversionMultiplier * SLOW_DRIVE_SPEED;
      double fieldAimDeg = stateMachine.getPassAimAngleDeg();
      double aimDeg = fieldAimDeg;
      // CTRE operator perspective adds 180° for Red in ROTATION_LOCK; subtract to avoid double-inversion.
      if (!stateMachine.isBlueAlliance()) aimDeg -= 180.0;
      SmartDashboard.putNumber("aimDeg", aimDeg);
      SmartDashboard.putBoolean("usingSOTM", false);
      SmartDashboard.putNumber("robotHeadingDeg", drivetrain.getState().Pose.getRotation().getDegrees());
      driveAimed(xSpeed, ySpeed, fieldAimDeg, aimDeg);
    } else if (shootHeld && aimState == RobotState.SHOOTING) {
      boolean isInverted = SmartDashboard.getBoolean("driveInverted", false);
      double inversionMultiplier = isInverted ? -1.0 : 1.0;
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(
          controller, drivetrain, MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      double xSpeed = speeds.vxMetersPerSecond * OIConstants.MAX_SPEED_COEFFICIENT * inversionMultiplier * SLOW_DRIVE_SPEED;
      double ySpeed = speeds.vyMetersPerSecond * OIConstants.MAX_SPEED_COEFFICIENT * inversionMultiplier * SLOW_DRIVE_SPEED;

      var sotmResult = stateMachine.getLastSOTMResult();
      boolean usingSOTM = sotmResult.isValid();
      double fieldAimDeg = usingSOTM
          ? sotmResult.driveAngle().getDegrees()
          : stateMachine.getStaticAimAngleDeg();
      double aimDeg = fieldAimDeg;
      // CTRE operator perspective adds 180° for Red in ROTATION_LOCK; subtract to avoid double-inversion.
      if (!stateMachine.isBlueAlliance()) aimDeg -= 180.0;
      SmartDashboard.putNumber("aimDeg", aimDeg);
      SmartDashboard.putBoolean("usingSOTM", usingSOTM);
      SmartDashboard.putNumber("robotHeadingDeg", drivetrain.getState().Pose.getRotation().getDegrees());
      driveAimed(xSpeed, ySpeed, fieldAimDeg, aimDeg);
    } else if (currentPose != null && targetPose != null && (autoDrive || requestedPose != null)) {
      boolean isBlueAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
      final double directionMultiplier = isBlueAlliance ? -1.0 : 1.0;

      final Translation2d translationToPoint = currentPose.getTranslation().minus(targetPose.getTranslation());
      final double linearDistance = translationToPoint.getNorm();
      double ff = 0;
      if (linearDistance >= Units.inchesToMeters(3)) {
        ff = STATIC_FRICTION_CONSTANT * MAX_SPEED;
      }

      double multiplier = (driveToPointSpeedMultiplier != null)
          ? Math.max(0.0, Math.min(1.0, driveToPointSpeedMultiplier))
          : 1.0;
      double maxSpeed = MAX_DRIVE_TO_POINT_SPEED * multiplier * speedCoefficient;

      final Rotation2d directionOfTravel = translationToPoint.getAngle();
      final double velocity = Math.min(Math.abs(driveToPointController.calculate(linearDistance, 0)) + ff, maxSpeed);
      final double xSpeed = velocity * directionOfTravel.getCos() * directionMultiplier;
      final double ySpeed = velocity * directionOfTravel.getSin() * directionMultiplier;

      drivetrain.drive(
          xSpeed,
          ySpeed,
          disableRotationLock ? 0.0 : targetPose.getRotation().getDegrees(),
          disableRotationLock ? Swerve.DriveMode.FIELD_RELATIVE : Swerve.DriveMode.DRIVE_TO_POINT);
    } else {

      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain,
          MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      final double coeff = speedCoefficient == 1.0 ? OIConstants.MAX_SPEED_COEFFICIENT : speedCoefficient;
      double xSpeed = speeds.vxMetersPerSecond * coeff;
      double ySpeed = speeds.vyMetersPerSecond * coeff;

      if (controller.leftTrigger(0.5).getAsBoolean()) {
        xSpeed = xSpeed * SLOW_DRIVE_SPEED;
        ySpeed = ySpeed * SLOW_DRIVE_SPEED;
      }

      boolean isInverted = SmartDashboard.getBoolean("driveInverted", false);
      double inversionMultiplier = isInverted ? -1.0 : 1.0;
      // System.out.println("isInverted: "+inversionMultiplier);

      drivetrain.drive(
          xSpeed * inversionMultiplier,
          ySpeed * inversionMultiplier,
          -speeds.omegaRadiansPerSecond,
          Swerve.DriveMode.FIELD_RELATIVE);
    }
  }

  /**
   * Drives while holding the aim heading. Turning in place leaves the wheels angled around the
   * robot (it looks like an X), and they'd stay that way once the turn finishes. So when the
   * driver isn't translating and the robot is on target, park the wheels straight instead, and
   * only go back to heading control if the heading drifts past AIM_RELEASE_DEG.
   *
   * @param fieldAimDeg heading to aim at, field frame (what the state machine checks)
   * @param requestAimDeg same heading, adjusted for CTRE's operator perspective
   */
  private void driveAimed(double xSpeed, double ySpeed, double fieldAimDeg, double requestAimDeg) {
    double errorDeg = Math.abs(MathUtil.inputModulus(
        drivetrain.getState().Pose.getRotation().getDegrees() - fieldAimDeg, -180.0, 180.0));
    // Same translational deadband the AIM request uses: below it the robot isn't moving.
    boolean stopped = Math.hypot(xSpeed, ySpeed) < Swerve.AIM_TRANSLATION_DEADBAND_MPS;
    if (!stopped || errorDeg > AIM_RELEASE_DEG) {
      wheelsParked = false;
    } else if (errorDeg < AIM_PARK_DEG) {
      wheelsParked = true;
    }
    if (wheelsParked) {
      drivetrain.drive(0.0, 0.0, 0.0, DriveMode.WHEELS_STRAIGHT);
    } else {
      drivetrain.drive(xSpeed, ySpeed, requestAimDeg, DriveMode.AIM);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (requestedPose != null) {
      return getIsInPosition(
          drivetrain.getState().Pose, requestedPose, drivetrain.getState().Speeds);
    }
    return false;
  }

  private double getFieldRelativeTargetRotation(double degrees) {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (alliance == Alliance.Red) {
      switch ((int) degrees) {
        case -90:
          return 90.0;
        case 135:
          return -45.0;
        case 90:
          return -90;
        case -135:
          return 45.0;
        default:
          return degrees;
      }
    } else {
      return degrees;
    }
  }

  private boolean getIsInPosition(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds speeds) {
    final Translation2d translationToPoint = currentPose.getTranslation().minus(targetPose.getTranslation());
    final double linearDistance = translationToPoint.getNorm();
    final double linearRotation = currentPose.getRotation().getDegrees()
        - getFieldRelativeTargetRotation(targetPose.getRotation().getDegrees());
    final double currentVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    final double tolerance = poseTolerance > 0 ? poseTolerance : 1;
    return linearDistance < Units.inchesToMeters(tolerance) && (Math.abs(linearRotation) < 10 || disableRotationLock);
  }

  /** Drive speed multiplier while intaking, shooting, or passing, so the driver can line up. */
  private static final double SLOW_DRIVE_SPEED = 0.4;

  /**
   * Stopped and aimed within this many degrees → park the wheels straight. Just above where the
   * heading controller's rotational deadband stops turning (~4°), well inside the 15° fire window.
   */
  private static final double AIM_PARK_DEG = 5.0;

  /** Parked wheels go back to heading control if the heading drifts past this. */
  private static final double AIM_RELEASE_DEG = 8.0;

  /** True while stopped on target with the wheels parked straight (see driveAimed). */
  private boolean wheelsParked = false;

  /**
   * Distance (m) inward from each side when targeting a corner (field: blue right
   * = 0,0, red left = max, max).
   */
  private static final double CORNER_TARGET_OFFSET_M = 1.0;

  /** Convenience: is the robot on the blue alliance? */
  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }
}