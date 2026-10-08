// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.SignalLogger;

// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ShotConstants;
import frc.robot.sim.IntakeSim;
import frc.robot.sim.MechanismSimManager;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import java.util.ArrayList;
import java.util.Collections;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Sim-only intake zone. Tracks held fuel and gates shot spawning. */
  private IntakeSim intakeSim = null;

  /** Sim-only physics for the flywheel, feeder, spindexer, intake roller, intake wrist, and hood. */
  private MechanismSimManager mechanismSim = null;

  /** Ticks until the next shot is allowed to spawn (~0.5 s @ 50 Hz). */
  private int simShotCooldown = 0;
  /** Consecutive ticks that readyToFire has been true; must reach SETTLE_TICKS before firing. */
  private int simReadySettleTicks = 0;
  private static final int SETTLE_TICKS = 50; // 1 second @ 50 Hz

  /** Fuel piece poses for live AdvantageScope field rendering (NT struct arrays). */
  private final StructArrayPublisher<Pose3d> simFuelPoses3dPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("MapleSim/Fuel3d", Pose3d.struct)
          .publish();
  private final StructArrayPublisher<Pose2d> simFuelPoses2dPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("MapleSim/Fuel2d", Pose2d.struct)
          .publish();

  public Robot() {
    SignalLogger.setPath("/U/ctre-logs/");
    if (RobotBase.isSimulation()) {
      // Override the arena before RobotContainer (which constructs Swerve and registers
      // the drivetrain with the arena). false = no ramp colliders so the robot can drive
      // over the field bumps instead of being stopped by them.
      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    }
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // DataLogManager.start();
    // DataLogManager.logNetworkTables(false);
    SmartDashboard.putBoolean("driveInverted",false);
    SmartDashboard.putBoolean("velocityLock", false);

    SmartDashboard.putBoolean("slowSpindexer", false);
    SmartDashboard.putBoolean("slowShooter", false);
    SmartDashboard.putBoolean("slowFeeder", false);
  }

  @Override
  public void robotPeriodic() {
    SignalLogger.writeDouble("Voltage", RobotController.getBatteryVoltage(), "V");
    SignalLogger.writeDouble("CAN Utilization", RobotController.getCANStatus().percentBusUtilization, "%");
    SignalLogger.writeDouble("CAN Error Count", RobotController.getCANStatus().receiveErrorCount, "#");
    m_robotContainer.setAuto();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    
    m_robotContainer.getStateMachine().setMatchStarted();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.getStateMachine().setMatchStarted();
    m_robotContainer.resetState();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationInit() {
    // Populate the Arena2026Rebuilt field with fuel balls and scoring targets.
    SimulatedArena.getInstance().resetFieldForAuto();

    var sm = m_robotContainer.getStateMachine();
    mechanismSim = new MechanismSimManager(sm);
    // Only collect fuel once the intake arm has physically swung down, not the instant the
    // state changes to INTAKING.
    intakeSim = new IntakeSim(
        sm.drivetrain.getDriveSimulation(),
        () -> sm.getCurrentState() == RobotState.INTAKING && mechanismSim.isIntakeDeployed());
  }

  @Override
  public void simulationPeriodic() {
    StateMachine sm = m_robotContainer.getStateMachine();

    // Step mechanism physics first so the intake-deployed check below sees this loop's arm angle.
    if (mechanismSim != null) mechanismSim.update();

    // Toggle the intake zone before the arena ticks so contact detection runs with
    // the correct extended/retracted state for this frame.
    if (intakeSim != null) intakeSim.update();

    // Advance MapleSim physics. Inside this call, each module's SimulatedMotorController
    // pushes encoder positions into the TalonFX/CANcoder sim states so CTRE's odometry
    // sees fresh sensor values.
    SimulatedArena.getInstance().simulationPeriodic();

    // Push MapleSim's gyro reading into the Pigeon2 sim state.
    sm.drivetrain.updateSimState();

    // Sync CTRE odometry to MapleSim's ground-truth pose so AdvantageScope shows
    // the robot at the same position where MapleSim runs collision/intake physics.
    sm.drivetrain.syncOdometryFromSim();

    maybeSpawnShot(sm);
    publishSimGamePieces(sm);
  }

  /** Publishes all simulated fuel poses so AdvantageScope can render moving game pieces live. */
  private void publishSimGamePieces(StateMachine sm) {
    Pose3d[] arenaFuel = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    ArrayList<Pose3d> allFuelPoses = new ArrayList<>(arenaFuel.length + 1);
    Collections.addAll(allFuelPoses, arenaFuel);

    if (intakeSim != null && intakeSim.getHeldCount() > 0) {
      allFuelPoses.add(getHeldFuelPose(sm));
    }

    Pose3d[] fuel3d = allFuelPoses.toArray(Pose3d[]::new);
    simFuelPoses3dPublisher.set(fuel3d);

    Pose2d[] fuel2d = new Pose2d[fuel3d.length];
    for (int i = 0; i < fuel3d.length; i++) {
      fuel2d[i] = fuel3d[i].toPose2d();
    }
    simFuelPoses2dPublisher.set(fuel2d);

    SmartDashboard.putNumber("simHeldFuelCount", intakeSim != null ? intakeSim.getHeldCount() : 0);
    SmartDashboard.putNumber("simFuelVisibleCount", fuel3d.length);
  }

  /** Approximate in-robot location for a held fuel piece, published for visualization. */
  private Pose3d getHeldFuelPose(StateMachine sm) {
    Pose2d robotPose = sm.drivetrain.getState().Pose;
    Translation2d heldOffsetRobot = new Translation2d(0.28, 0.0);
    Translation2d heldField = robotPose.getTranslation().plus(heldOffsetRobot.rotateBy(robotPose.getRotation()));
    return new Pose3d(
        new Translation3d(heldField.getX(), heldField.getY(), 0.35),
        new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
  }

  /**
   * Spawns a {@link RebuiltFuelOnFly} projectile when the state machine is firing and the
   * intake holds a fuel ball. Cooldown rate-limits to one shot per ~0.5 s.
   */
  private void maybeSpawnShot(StateMachine sm) {
    boolean isShooting = sm.getCurrentState() == RobotState.SHOOTING;
    boolean isPassing = sm.getCurrentState() == RobotState.PASSING;
    if (!isShooting && !isPassing) {
      simShotCooldown = 0;
      simReadySettleTicks = 0;
      return;
    }
    // Mirror the real robot's gate: only fire when heading and speed are both OK.
    if (!sm.isReadyToFire()) {
      simReadySettleTicks = 0;
      return;
    }
    // Require the robot to stay on-target for SETTLE_TICKS before spawning the shot.
    if (simReadySettleTicks < SETTLE_TICKS) {
      simReadySettleTicks++;
      return;
    }
    if (simShotCooldown > 0) {
      simShotCooldown--;
      return;
    }
    if (intakeSim == null || intakeSim.getHeldCount() == 0) return;

    Pose2d pose = sm.drivetrain.getState().Pose;
    ChassisSpeeds robotRelativeSpeeds = sm.drivetrain.getState().Speeds;
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());
    double headingRad;
    double rps;
    double tof;

    if (isPassing) {
      // Passing uses a fixed aim angle; SOTM result may not be valid.
      headingRad = Math.toRadians(sm.getPassAimAngleDeg()) + Math.PI;
      double dist =
          pose.getTranslation()
              .minus(
                  sm.isBlueAlliance() ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED)
              .getNorm();
      rps = ShotConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(dist);
      tof = 0.75;
    } else {
      var result = sm.getLastSOTMResult();
      if (!result.isValid()) return;
      // Launch at the speed the simulated flywheel is ACTUALLY spinning, not the setpoint, so
      // shots fired during spin-up fall short like they would on the real robot.
      rps = mechanismSim != null ? mechanismSim.getFlywheelVelocityRps() : result.shooterRps();
      tof = result.timeOfFlightSec() > 0 ? result.timeOfFlightSec() : 0.75;
      // Rear of robot = pose.getRotation() + PI (physically where the shooter points).
      headingRad = pose.getRotation().getRadians() + Math.PI;
    }

    // Decompose total launch speed into horizontal + vertical from time-of-flight arc.
    // RPS_TO_SPEED is calibrated against the real shooter LUT — tune alongside it.
    final double RPS_TO_SPEED = 0.175;
    double totalSpeed = rps * RPS_TO_SPEED;
    double vertSpeed = (9.81 * tof) / 2.0;
    double horzSpeed = Math.sqrt(Math.max(0, totalSpeed * totalSpeed - vertSpeed * vertSpeed));
    double pitchRad = Math.atan2(vertSpeed, horzSpeed);

    intakeSim.consume();

    // RebuiltFuelOnFly rotates shooterPositionOnRobot by shooterFacing internally, not by
    // chassisFacing. For a rear shooter (shooterFacing = pose.rotation + π), passing
    // (+0.20, 0) lands the spawn 0.20 m behind the chassis center.
    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                pose.getTranslation(),
                new Translation2d(0.20, 0),
                fieldRelativeSpeeds,
                Rotation2d.fromRadians(headingRad),
                Meters.of(0.80),
                MetersPerSecond.of(totalSpeed),
                Radians.of(pitchRad)));

    simShotCooldown = 25;
    simReadySettleTicks = 0;
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
