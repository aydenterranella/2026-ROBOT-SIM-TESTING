// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.firecontrol.ShotCalculator;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.subsystems.PositionSubsystem;
import frc.lib.team9410.subsystems.VelocitySubsystem;
import frc.lib.team9410.subsystems.VelocityTorqueSubsystem;
import frc.robot.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.ShotConstants;
import frc.robot.utils.FieldUtils.GameZone;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LimelightHelpers;

// Subsystem that holds high-level robot state and drives transitions.
public class StateMachine extends SubsystemBase {

  public enum RobotState {
    READY,
    INTAKING,
    OUTTAKING,
    SHOOTING,
    PASSING
  }

  private RobotState wantedState = RobotState.READY;
  private RobotState currentState = RobotState.READY;
  private RobotState previousState = RobotState.READY;

  // Position subsystems
  public final PositionSubsystem shooterHood = new PositionSubsystem(Constants.Shooter.HOOD_CONFIG);
  public final PositionSubsystem intakeWrist = new PositionSubsystem(Constants.Intake.WRIST_CONFIG);

  // Velocity subsystems
  public final VelocitySubsystem shooter = new VelocitySubsystem(Constants.Shooter.FLYWHEEL_CONFIG);
  public final VelocityTorqueSubsystem intakeRoller = new VelocityTorqueSubsystem(Constants.Intake.ROLLER_CONFIG);
  public final VelocitySubsystem spindexer = new VelocitySubsystem(Constants.Spindexer.SPINDEXER_CONFIG);
  public final VelocitySubsystem feeder = new VelocitySubsystem(Constants.Feeder.FEEDER_CONFIG);

  private final Vision vision = new Vision();
  @SuppressWarnings("unused")
  private final Dashboard dashboard = new Dashboard();

  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  // Shoot-on-the-move fire control
  private final ShotCalculator shotCalc;
  private ShotCalculator.LaunchParameters lastSOTMResult = ShotCalculator.LaunchParameters.INVALID;
  // Static aim angle (degrees) toward current target; updated each shooting cycle.
  private double staticAimAngleDeg = 0.0;
  // Fixed pass-aim heading (degrees, field frame) toward own alliance side.
  private double passAimAngleDeg = 0.0;
  // True when heading and shooter speed are both within tolerance — set by runShootingToTarget.
  private boolean readyToFire = false;

  // NT pose publishing
  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Robot/Pose", Pose2d.struct)
          .publish();
  private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Robot/SwerveStates", SwerveModuleState.struct)
          .publish();

  private boolean gyroReset = false;

  private int intakeTimer = 0;

  private String bestLimelight = "";

  private boolean matchStarted = false;


  public StateMachine() {
    ShotCalculator.Config cfg = new ShotCalculator.Config();
    cfg.launcherOffsetX = -0.20;        // negative — shooter is behind robot center
    cfg.launcherOffsetY = 0.0;
    cfg.shooterAngleOffsetRad = Math.PI; // shooter faces rear
    cfg.maxSOTMSpeed = 3.0;
    cfg.minScoringDistance = 0.5;
    cfg.maxScoringDistance = 5.0;
    shotCalc = new ShotCalculator(cfg);
    shotCalc.loadShotLUT(ShotConstants.SHOT_LUT);
  }

  @Override
  public void periodic() {
    handleStateTransitions();
    executeState();
    PowerRobotContainer.setData("robotState", currentState.name());
    PowerRobotContainer.setData("currentZone", getZoneFromPRC());
    setRobotPose();
    // logPoseToWpilog(drivetrain.getState().Pose);
    
  
    // Map<String, Object> robotData = PowerRobotContainer.getAllData();
    // SmartDashboard.putData("robotData", builder -> {
    // builder.setSmartDashboardType("robotData");
    // for (String key : robotData.keySet()) {
    //     builder.addDoubleProperty(key, () -> robotData.get(key), v -> robotData.put(key, v));}});

  
    SmartDashboard.putString("gameZone", FieldUtils.getZone(drivetrain.getState().Pose).name());
    SmartDashboard.putNumber("passAimDeg", passAimAngleDeg);
    posePublisher.set(drivetrain.getState().Pose);
    moduleStatePublisher.set(drivetrain.getState().ModuleStates);

    // PowerRobotContainer.setData("SpindexerVelocity", spindexer.getVelocityMotor().getVelocity().getValueAsDouble());
    // PowerRobotContainer.setData("ShooterVelocity", shooter.getVelocityMotor().getVelocity().getValueAsDouble());
    // PowerRobotContainer.setData("FeederVelocity", feeder.getVelocityMotor().getVelocity().getValueAsDouble());
  }

  public void resetGyro () {
    gyroReset = false;
  }

  // private void logPoseToWpilog(Pose2d pose) {
  //   if (m_odometryLog == null) {
  //     var log = DataLogManager.getLog();
  //     m_odometryLog = StructLogEntry.create(log, "Odometry", Pose2d.struct);
  //     m_robotPositionLog = StructLogEntry.create(log, "RobotPosition", Translation2d.struct);
  //   }
  //   m_odometryLog.append(pose);
  //   m_robotPositionLog.append(pose.getTranslation());
  // }

  public void setRobotPose () {
    // In simulation, MapleSim/CTRE provide odometry truth. Vision fusion can introduce
    // jitter when no simulated Limelight pipeline is configured.
    if (RobotBase.isSimulation()) {
      SmartDashboard.putNumber("currentPoseX", drivetrain.getState().Pose.getX());
      SmartDashboard.putNumber("currentPoseY", drivetrain.getState().Pose.getY());
      SmartDashboard.putNumber("currentPoseYaw", drivetrain.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putString("bestLimelight", "sim");
      return;
    }

    bestLimelight = vision.getBestLimelight();

    // Skip vision when no Limelight has valid data.
    if (bestLimelight == null || bestLimelight.isEmpty()) {
    } else {
      // MegaTag2 needs gyro yaw (not vision yaw) to resolve pose correctly on both alliances.
      double gyroYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();
      double pitchDeg   = drivetrain.getPigeon2().getPitch().getValueAsDouble();
      double rollDeg    = drivetrain.getPigeon2().getRoll().getValueAsDouble();
      LimelightHelpers.SetRobotOrientation(Constants.Vision.LEFT_TABLE,   gyroYawDeg, 0, pitchDeg, 0, rollDeg, 0);
      LimelightHelpers.SetRobotOrientation(Constants.Vision.RIGHT_TABLE,  gyroYawDeg, 0, pitchDeg, 0, rollDeg, 0);
      LimelightHelpers.SetRobotOrientation(Constants.Vision.SHOOTER_TABLE, gyroYawDeg, 0, pitchDeg, 0, rollDeg, 0);

      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
      if (mt2 != null && mt2.tagCount > 0) {
        Pose2d newPose = mt2.pose;
        // Reject out-of-bounds startup garbage poses.
        double x = newPose.getX();
        double y = newPose.getY();
        if (x < Constants.Field.X_MIN - Constants.Field.TOL || x > Constants.Field.X_MAX + Constants.Field.TOL
            || y < Constants.Field.Y_MIN - Constants.Field.TOL || y > Constants.Field.Y_MAX + Constants.Field.TOL) {
          // Bad pose; do not reset or add to estimator.
        } else {
          // System.out.println(matchStarted);
          if (!gyroReset || !matchStarted) {
            // newPose = new Pose2d(newPose.getX(), newPose.getY(), drivetrain.getState().Pose.getRotation());
            drivetrain.resetPose(newPose);
            gyroReset = true;
          }
          // else {
          //   newPose = new Pose2d(newPose.getX(), newPose.getY(), drivetrain.getState().Pose.getRotation());
          // }
          // drivetrain.resetPose(newPose);
          drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
          drivetrain.addVisionMeasurement(newPose, mt2.timestampSeconds);
        }
      }
    }

      // PowerRobotContainer.setData("distanceToHopper", linearDistance);
      SmartDashboard.putNumber("currentPoseX", drivetrain.getState().Pose.getX());
      SmartDashboard.putNumber("currentPoseY", drivetrain.getState().Pose.getY());
      SmartDashboard.putNumber("currentPoseYaw", drivetrain.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putString("bestLimelight", bestLimelight);
  }

  private void handleStateTransitions() {
    if (currentState != wantedState) {
      previousState = currentState;
      if (canTransitionTo(wantedState)) {
        currentState = wantedState;
        // Letting go of intake lifts the arm back to idle. Done once on the transition, not every
        // loop in READY, so autos can still hold the intake down while READY.
        if (previousState == RobotState.INTAKING && currentState == RobotState.READY) {
          intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
        }
      }
    }
  }

  /** Override to add transition guards (e.g. require subsystems ready). */
  private boolean canTransitionTo(RobotState target) {
    return true;
  }

  private void executeState() {
    switch (currentState) {
      case READY:
        executeReady();
        break;
      case INTAKING:
        executeIntaking();
        break;
      case OUTTAKING:
        executeOuttaking();
        break;
      case SHOOTING:
        executeShooting();
        break;
      case PASSING:
        executePassing();
        break;
      default:
        break;
    }
  }

  private void executeReady() {
    shooter.brake();
    spindexer.brake();
    feeder.brake();
    if (intakeWrist.getSetpointRotations() > Constants.Intake.INTAKE_IDLE){
      intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
    }
    intakeTimer = 0;
  }

  private void executeIntaking() {
    shooter.brake();
    spindexer.brake();
    feeder.brake();
    intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
    intakeRoller.setVelocity(145);
    intakeTimer = 0;
  }

  private void executeOuttaking() {
    shooter.brake();
    spindexer.brake();
    feeder.brake();
    intakeWrist.setPositionRotations(Constants.Intake.INTAKE_FEED);
    intakeRoller.setVelocity(-145);
    intakeTimer = 0;
  }

  private void executePassing() {
    intakeTimer++;
    intakeRoller.brake();
    // Blue hopper is to the LEFT (low X): robot faces RIGHT (0°) so rear aims left.
    // Red hopper is to the RIGHT (high X): robot faces LEFT (180°) so rear aims right.
    passAimAngleDeg = isBlueAlliance() ? 0.0 : 180.0;
    Translation2d target = isBlueAlliance() ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED;
    runShootingToTarget(target, passAimAngleDeg);
    if (intakeWrist.getSetpointRotations() >= Constants.Intake.INTAKE_IDLE) {
      if (intakeTimer % 50 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_FEED);
      } else if (intakeTimer % 25 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
      }
    }
  }

  public GameZone getZoneFromPRC () {
    Pose2d pose = drivetrain.getState().Pose;

    return FieldUtils.getZone(pose);
  }

  private void executeShooting() {
    intakeTimer++;
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    boolean inOurZone = getAllianceZone() == zone;

    Translation2d target = inOurZone
        ? (zone == GameZone.BLUE_ALLIANCE ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED)
        : getTargetCornerLocation();

    runShootingToTarget(target);
    if (intakeWrist.getSetpointRotations() >= Constants.Intake.INTAKE_IDLE) {
      if (intakeTimer % 50 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_FEED);
      } else if (intakeTimer % 25 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
      }
    }
  }

  private void runShootingToTarget(Translation2d target) {
    runShootingToTarget(target, Double.NaN);
  }

  // Run shooter, hood, feeder, and spindexer for the current target.
  // fixedHeadingDeg: if not NaN, overrides SOTM/static for the heading gate (used in PASSING).
  private void runShootingToTarget(Translation2d target, double fixedHeadingDeg) {
    Pose2d pose = drivetrain.getState().Pose;
    double distance = pose.getTranslation().minus(target).getNorm();
    SmartDashboard.putNumber("distanceToHopper", distance);

    // SOTM solver: convert robot-relative speeds into field frame.
    ChassisSpeeds robotVel = drivetrain.getState().Speeds;
    ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, pose.getRotation());

    // (0,0) disables behind-hub rejection for the 2026 hopper target.
    Translation2d hubForward = new Translation2d(0, 0);

    LimelightHelpers.PoseEstimate latestMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
    double visionConf = (latestMT2 != null && latestMT2.tagCount > 0) ? 1.0 : 0.5;
    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        pose, fieldVel, robotVel, target, hubForward, visionConf);
    lastSOTMResult = shotCalc.calculate(inputs);

    // Choose shooter speed and hood angle from SOTM or fallback LUT.
    double shooterVelo;
    double hoodPos;
    if (lastSOTMResult.isValid()) {
      shooterVelo = lastSOTMResult.shooterRps();
      // ShotCalculator returns degrees; convert to rotations for the motor controller.
      double hoodDeg = shotCalc.getHoodAngle(lastSOTMResult.solvedDistanceM());
      hoodPos = hoodDeg / 360.0;
    } else {
      shooterVelo = ShotConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance);
      hoodPos = ShotConstants.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(distance);
    }

    // Update fallback heading used when SOTM is invalid.
    staticAimAngleDeg = Math.toDegrees(Math.atan2(
        target.getY() - pose.getY(), target.getX() - pose.getX())) + 180.0;

    double actualShooterVelo = shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble();
    SmartDashboard.putNumber("shooterVelocity", shooterVelo);
    SmartDashboard.putNumber("shooterVelocityActual", actualShooterVelo);
    SmartDashboard.putNumber("shooterVelocityError", shooterVelo - actualShooterVelo);
    SmartDashboard.putNumber("shooterHoodPos", hoodPos);
    SmartDashboard.putNumber("shooterHoodPosActual", shooterHood.getPositionRotations());
    SmartDashboard.putNumber("rpmOffset", shotCalc.getOffset());
    SmartDashboard.putNumber("sotmConfidence", lastSOTMResult.confidence());
    SmartDashboard.putBoolean("sotmValid", lastSOTMResult.isValid());

    boolean slowSpindexer = SmartDashboard.getBoolean("slowSpindexer", false);

    boolean slowFeeder    = SmartDashboard.getBoolean("slowFeeder", false);

    // if (!slowShooter) {
    //   shooterVelo += 1; // old tuning hack — disabled, FireControl already sets correct RPM
    // }

    shooter.setVelocity(shooterVelo);
    shooterHood.setPositionRotations(
        MathUtil.clamp(hoodPos, ShooterConstants.SHOOTER_HOOD_MIN, ShooterConstants.SHOOTER_HOOD_MAX));

    // Heading gate: only feed/spin when robot is aimed within tolerance
    double targetHeadingRad;
    boolean usingFixedHeading = !Double.isNaN(fixedHeadingDeg);
    if (usingFixedHeading) {
      targetHeadingRad = Math.toRadians(fixedHeadingDeg);
    } else if (lastSOTMResult.isValid()) {
      targetHeadingRad = lastSOTMResult.driveAngle().getRadians();
    } else {
      targetHeadingRad = Math.toRadians(staticAimAngleDeg);
    }
    double headingErrorDeg = Math.abs(Math.toDegrees(
        MathUtil.angleModulus(pose.getRotation().getRadians() - targetHeadingRad)));
    boolean headingOK = headingErrorDeg < ((!usingFixedHeading && lastSOTMResult.isValid()) ? 15.0 : 5.0);
    SmartDashboard.putNumber("headingErrorDeg", headingErrorDeg);
    SmartDashboard.putBoolean("headingOK", headingOK);

    // double velocityThreshold = TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance) - 1.5; // old — fought FireControl
    double velocityThreshold = shooterVelo - 1.5;
    // The flywheel is physically simulated now (frc.robot.sim.MechanismSimManager), so sim uses
    // the same at-speed check as the real robot instead of skipping it.
    boolean shooterAtSpeed =
        shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble() > velocityThreshold;

    SmartDashboard.putBoolean("shooterAtSpeed", shooterAtSpeed);
    readyToFire = headingOK && shooterAtSpeed;
    SmartDashboard.putBoolean("readyToFire", readyToFire);

    if (readyToFire) {
      feeder.setVelocity(slowFeeder ? -60 : -2 * shooterVelo);
      spindexer.setVelocity(slowSpindexer ? 70 : 80);
    } else {
      feeder.brake();
      spindexer.brake();
    }
  }

  // True when heading and shooter speed are both within tolerance this cycle.
  public boolean isReadyToFire() {
    return readyToFire;
  }

  // Latest SOTM fire-control result. Updated every shooting cycle. */
  public ShotCalculator.LaunchParameters getLastSOTMResult() {
    return lastSOTMResult;
  }

  // The ShotCalculator instance (used by simulation to query hood angles etc.). */
  public ShotCalculator getShotCalc() {
    return shotCalc;
  }

  // Static aim angle (degrees, field frame) toward the current target. Used as heading fallback.
  public double getStaticAimAngleDeg() {
    return staticAimAngleDeg;
  }

  // Pass aim angle (degrees, field frame) toward our alliance side.
  public double getPassAimAngleDeg() {
    return passAimAngleDeg;
  }

  public void setWantedState(RobotState state) {
    wantedState = state;
  }

  public RobotState getCurrentState() {
    return currentState;
  }

  public RobotState getWantedState() {
    return wantedState;
  }

  public RobotState getPreviousState() {
    return previousState;
  }

  /** Convenience: is the robot on the blue alliance? */
  public boolean isBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  /** Distance (m) inward from each side when targeting a corner (field: blue right = 0,0, red left = max, max). */
  private static final double CORNER_TARGET_OFFSET_M = 1.0;

  /**
   * Returns a target position 1 m inward from each side that forms the corner (end wall and side wall).
   * Blue right = (0,0), red left = (max, max). Picks the nearest of our alliance's two corners.
   */
  public Translation2d getTargetCornerLocation() {
    Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
    if (isBlueAlliance()) {
      // Blue corners: 1m from blue wall (low X) and 1m from bottom (low Y) or top (high Y)
      Translation2d fromBottom = new Translation2d(
          Constants.Field.BLUE_BOTTOM_CORNER.getX() + CORNER_TARGET_OFFSET_M,
          Constants.Field.BLUE_BOTTOM_CORNER.getY() + CORNER_TARGET_OFFSET_M);
      Translation2d fromTop = new Translation2d(
          Constants.Field.BLUE_TOP_CORNER.getX() + CORNER_TARGET_OFFSET_M,
          Constants.Field.BLUE_TOP_CORNER.getY() - CORNER_TARGET_OFFSET_M);
      return robotPos.getY() > 4 ? fromTop : fromBottom;
    } else {
      // Red corners: 1m from red wall (high X) and 1m from bottom (low Y) or top (high Y)
      Translation2d fromBottom = new Translation2d(
          Constants.Field.RED_BOTTOM_CORNER.getX() - CORNER_TARGET_OFFSET_M,
          Constants.Field.RED_BOTTOM_CORNER.getY() + CORNER_TARGET_OFFSET_M);
      Translation2d fromTop = new Translation2d(
          Constants.Field.RED_TOP_CORNER.getX() - CORNER_TARGET_OFFSET_M,
          Constants.Field.RED_TOP_CORNER.getY() - CORNER_TARGET_OFFSET_M);
      // System.out.println(robotPos.getY() > 4 ? fromTop : fromBottom);
      return robotPos.getY() > 4 ? fromTop : fromBottom;
    }
  }

  public GameZone getAllianceZone() {
    if (isBlueAlliance()) {
      return GameZone.BLUE_ALLIANCE;
    } else {
      return GameZone.RED_ALLIANCE;
    }
  }


  // thanks wpilib
  // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public void setMatchStarted () {
    matchStarted = true;
  }
}
