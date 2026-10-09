package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.sim.MechanismSimConstants;
import frc.robot.sim.PhoenixSimUtil;
import frc.robot.sim.RobotBumpSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
  /** MapleSim drivetrain simulation. Non-null only when {@link Utils#isSimulation()}. */
  private SwerveDriveSimulation m_simDrivetrain = null;
  /** Bump traversal sim (Z/pitch/roll + ramp sliding). Non-null only in simulation. */
  private RobotBumpSim m_bumpSim = null;
  /** Latest 3D sim pose from {@link #m_bumpSim}; flat until the first update. */
  private Pose3d m_simPose3d = Pose3d.kZero;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  private final PhoenixPIDController HEADING_CONTROLLER = new PhoenixPIDController(6.5, 0, 0.25);

  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public double MAX_ANGULAR_RATE =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit

  public final SwerveRequest.FieldCentric FIELD_RELATIVE =
      new SwerveRequest.FieldCentric()
          .withDeadband(MAX_SPEED * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public final SwerveRequest.RobotCentric ROBOT_RELATIVE =
      new SwerveRequest.RobotCentric()
          .withDeadband(MAX_SPEED * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public final SwerveRequest.FieldCentricFacingAngle DRIVE_AT_ANGLE =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(MAX_SPEED * 0.1)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
          .withDriveRequestType(DriveRequestType.Velocity);

  /**
   * Like DRIVE_AT_ANGLE, for driver-controlled aiming while shooting (DriveMode.AIM). The joystick already has its
   * own 10% deadband, and while shooting the stick is scaled down, so DRIVE_AT_ANGLE's 10%-of-max
   * deadband would swallow most of the stick: the robot sat still and only turned (wheels in the
   * turn-in-place "X"). This one only drops near-zero speeds.
   */
  public final SwerveRequest.FieldCentricFacingAngle AIM_AT_ANGLE =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(AIM_TRANSLATION_DEADBAND_MPS)
          .withRotationalDeadband(MAX_ANGULAR_RATE * 0.1)
          .withHeadingPID(6.5, 0, 0.25)
          .withDriveRequestType(DriveRequestType.Velocity);

  /** Below this commanded speed (m/s) an aiming robot counts as stopped. */
  public static final double AIM_TRANSLATION_DEADBAND_MPS = 0.02;

  private final SwerveRequest.SwerveDriveBrake BRAKE = new SwerveRequest.SwerveDriveBrake();
  /** Stopped with every wheel pointed straight ahead (robot-relative). */
  private final SwerveRequest.PointWheelsAt WHEELS_STRAIGHT =
      new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.kZero);

  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds();

  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(7), // Use dynamic voltage of 7 V
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
   */
  private final SysIdRoutine m_sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              /* This is in radians per second², but SysId only supports "volts per second" */
              Volts.of(Math.PI / 6).per(Second),
              /* This is in radians per second, but SysId only supports "volts" */
              Volts.of(Math.PI),
              null, // Use default timeout (10 s)
              // Log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, regulateModulesForSim(modules));
    if (Utils.isSimulation()) {
      initMapleSim();
    }
    DRIVE_AT_ANGLE.HeadingController = HEADING_CONTROLLER;
    DRIVE_AT_ANGLE.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, regulateModulesForSim(modules));
    if (Utils.isSimulation()) {
      initMapleSim();
    }
    DRIVE_AT_ANGLE.HeadingController = HEADING_CONTROLLER;
    DRIVE_AT_ANGLE.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    configureAutoBuilder();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public Swerve(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        regulateModulesForSim(modules));
    if (Utils.isSimulation()) {
      initMapleSim();
    }
    DRIVE_AT_ANGLE.HeadingController = HEADING_CONTROLLER;
    DRIVE_AT_ANGLE.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    configureAutoBuilder();
  }

  /** No-op on real robot; in sim strips encoder offsets + inversions so MapleSim can drive
   * the wheels from a clean zero. Must be static because it runs in {@code super(...)}. */
  private static SwerveModuleConstants<?, ?, ?>[] regulateModulesForSim(
      SwerveModuleConstants<?, ?, ?>[] modules) {
    if (!Utils.isSimulation()) return modules;
    for (int i = 0; i < modules.length; i++) {
      modules[i] = PhoenixSimUtil.regulateForSimulation(modules[i]);
    }
    return modules;
  }

  public void applyRequest(SwerveRequest request) {
    setControl(request);
  }

  public void drive(double x, double y, double rotation, DriveMode mode) {

    switch (mode) {
      case SYS_ID:
        break;
      case FIELD_RELATIVE:
        applyRequest(
            FIELD_RELATIVE.withVelocityX(x).withVelocityY(y).withRotationalRate(-rotation));
        break;
      case ROBOT_RELATIVE:
        applyRequest(
            ROBOT_RELATIVE.withVelocityX(x).withVelocityY(y).withRotationalRate(-rotation));
        break;
      case ROTATION_LOCK:
        applyRequest(
            DRIVE_AT_ANGLE
                .withVelocityX(x)
                .withVelocityY(y)
                .withTargetDirection(Rotation2d.fromDegrees(rotation)));
        break;
      case AIM:
        applyRequest(
            AIM_AT_ANGLE
                .withVelocityX(x)
                .withVelocityY(y)
                .withTargetDirection(Rotation2d.fromDegrees(rotation)));
        break;
      case DRIVE_TO_POINT:
        applyRequest(
            DRIVE_AT_ANGLE
                .withVelocityX(x)
                .withVelocityY(y)
                .withTargetDirection(Rotation2d.fromDegrees(rotation))
                .withMaxAbsRotationalRate(MAX_DRIVE_TO_POINT_ANGULAR_RATE));
        break;
      case BRAKE:
        applyRequest(BRAKE);
        break;
      case WHEELS_STRAIGHT:
        applyRequest(WHEELS_STRAIGHT);
        break;
    }
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
              });
    }
  }

  /**
   * Builds the MapleSim drivetrain and bridges its module/gyro state to Phoenix 6 sim states.
   *
   * <p>Replaces the CTRE Phoenix sim notifier loop. MapleSim's own arena tick is now the
   * authoritative source of motor + chassis physics. Each sub-tick, MapleSim pushes encoder
   * positions/velocities into the TalonFX/CANcoder sim states via {@link PhoenixSimUtil},
   * and reads back the voltage CTRE wants applied. The Pigeon2 yaw is pushed once per robot
   * loop from {@link #updateSimState()}.
   */
  private void initMapleSim() {
    // Gear ratios, wheel radius, and friction voltages mirror TunerConstants.
    SwerveModuleSimulationConfig moduleConfig =
        new SwerveModuleSimulationConfig(
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            5.4,
            12.1,
            Volts.of(0.2),
            Volts.of(0.2),
            Inches.of(2),
            KilogramSquareMeters.of(0.05),
            COTS.WHEELS.COLSONS.cof);

    // Module translations match TunerConstants module positions (±11.5" × ±10.5").
    Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(Inches.of(11.5), Inches.of(10.5)),
          new Translation2d(Inches.of(11.5), Inches.of(-10.5)),
          new Translation2d(Inches.of(-11.5), Inches.of(10.5)),
          new Translation2d(Inches.of(-11.5), Inches.of(-10.5)),
        };

    DriveTrainSimulationConfig driveConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(55))
            .withBumperSize(Inches.of(30), Inches.of(28))
            .withCustomModuleTranslations(moduleTranslations)
            .withSwerveModule(moduleConfig)
            .withGyro(COTS.ofPigeon2());

    // Spawn on the blue side; Robot.simulationPeriodic moves it to the right alliance once the DS
    // reports one, and auto routines reset via resetPose().
    m_simDrivetrain = new SwerveDriveSimulation(driveConfig, MechanismSimConstants.BLUE_SPAWN_POSE);
    SimulatedArena.getInstance().addDriveTrainSimulation(m_simDrivetrain);
    m_bumpSim = new RobotBumpSim(moduleTranslations);

    SwerveModuleSimulation[] simModules = m_simDrivetrain.getModules();
    for (int i = 0; i < 4; i++) {
      var ctreModule = getModule(i);
      simModules[i].useDriveMotorController(
          new PhoenixSimUtil.TalonFXMotorControllerSim(ctreModule.getDriveMotor()));
      simModules[i].useSteerMotorController(
          new PhoenixSimUtil.TalonFXWithRemoteCANcoderSim(
              ctreModule.getSteerMotor(), ctreModule.getEncoder()));
    }
  }

  /**
   * Pushes MapleSim's gyro reading into the Pigeon2 sim state. Call once per
   * {@code Robot.simulationPeriodic()} after {@code SimulatedArena.simulationPeriodic()}.
   */
  public void updateSimState() {
    if (m_simDrivetrain == null) return;
    Rotation2d gyroReading = m_simDrivetrain.getGyroSimulation().getGyroReading();
    getPigeon2().getSimState().setRawYaw(gyroReading.getMeasure());
  }

  /**
   * Steps the bump sim against MapleSim's latest pose and, while on a ramp, hands the
   * frictionless-slide X back to MapleSim so the robot physically slides off a bump it can't
   * crest. Call after {@code SimulatedArena.simulationPeriodic()} and before
   * {@link #syncOdometryFromSim()}.
   */
  public void updateBumpSim() {
    if (m_simDrivetrain == null) return;
    Pose2d simPose = m_simDrivetrain.getSimulatedDriveTrainPose();
    ChassisSpeeds fieldSpeeds = m_simDrivetrain.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    m_simPose3d =
        m_bumpSim.update(simPose, fieldSpeeds, SimulatedArena.getSimulationSubTicksIn1Period());

    if (m_bumpSim.isOnRamp()) {
      m_simDrivetrain.setSimulationWorldPose(m_bumpSim.getSimWorldPose(simPose));
      // setSimulationWorldPose zeroes linear velocity; restore it so MapleSim keeps the
      // driver's lateral momentum and exits the ramp at the bump sim's X velocity.
      m_simDrivetrain.setRobotSpeeds(
          new ChassisSpeeds(
              m_bumpSim.getSimXVelocity(),
              fieldSpeeds.vyMetersPerSecond,
              fieldSpeeds.omegaRadiansPerSecond));
    }
  }

  /** Returns the simulated 3D robot pose (with bump Z/pitch/roll), or {@code null} on the real robot. */
  public Pose3d getSimPose3d() {
    return m_simDrivetrain == null ? null : m_simPose3d;
  }

  /** True while the bump sim owns the robot's X (climbing/sliding on a ramp). */
  public boolean isSimOnBump() {
    return m_bumpSim != null && m_bumpSim.isOnRamp();
  }

  /** Returns the MapleSim drivetrain simulation, or {@code null} on the real robot. */
  public SwerveDriveSimulation getDriveSimulation() {
    return m_simDrivetrain;
  }

  /**
   * Returns MapleSim's ground-truth robot pose, or {@code null} on the real robot.
   * This is the authoritative position used by MapleSim's collision/intake physics.
   */
  public Pose2d getSimWorldPose() {
    if (m_simDrivetrain == null) return null;
    return m_simDrivetrain.getSimulatedDriveTrainPose();
  }

  /**
   * Seeds CTRE odometry from MapleSim's ground-truth pose without touching MapleSim's
   * physics body (avoids zeroing velocity). Call after each arena tick.
   */
  public void syncOdometryFromSim() {
    if (m_simDrivetrain == null) return;
    super.resetPose(m_simDrivetrain.getSimulatedDriveTrainPose());
  }

  /**
   * Resets only the pose estimate (e.g. to a vision pose). Same as {@link #resetPose} on the real
   * robot; in sim it leaves MapleSim's robot where it is instead of teleporting it to the estimate.
   */
  public void resetEstimatedPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
    if (m_simDrivetrain != null) {
      m_simDrivetrain.setSimulationWorldPose(pose);
      // Teleporting invalidates the ramp state; a stale X would snap the robot back.
      m_bumpSim.reset();
      // MapleSim's gyro only integrates turning, so point it at the new heading too.
      m_simDrivetrain.getGyroSimulation().setRotation(pose.getRotation());
    }
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * #setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  public static enum DriveMode {
    ROBOT_RELATIVE,
    FIELD_RELATIVE,
    SYS_ID,
    ROTATION_LOCK,
    /** Driver aiming while shooting/passing: like ROTATION_LOCK with a near-zero deadband. */
    AIM,
    DRIVE_TO_POINT,
    BRAKE,
    WHEELS_STRAIGHT
  }

  public void configureAutoBuilder() {
    try {
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> getState().Pose, // Supplier of current robot pose
          this::resetPose, // Consumer for seeding pose against auto
          () -> getState().Speeds, // Supplier of current robot speeds
          // Consumer of ChassisSpeeds and feedforwards to drive the robot
          (speeds, feedforwards) ->
              setControl(
                  m_pathApplyRobotSpeeds
                      .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                      .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                      .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(2.2, 0, 0),
              // PID constants for rotation
              new PIDConstants(7.0, 0, 0)),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }
}