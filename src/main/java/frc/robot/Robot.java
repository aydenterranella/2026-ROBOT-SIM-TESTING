// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.SignalLogger;

// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.FuelBumpSim;
import frc.robot.sim.FuelPathSim;
import frc.robot.sim.HopperSim;
import frc.robot.sim.IntakeSim;
import frc.robot.sim.MechanismSimConstants;
import frc.robot.sim.MechanismSimManager;
import frc.robot.sim.SimShot;
import frc.robot.sim.VisionSim;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.utils.JoystickPublisher;
import java.util.ArrayList;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Sends controller inputs to NT so AdvantageScope's Joysticks tab can show them live. */
  private final JoystickPublisher joystickPublisher = new JoystickPublisher();

  /** Sim-only intake: picks fuel up off the field and hands it to fuelPathSim. */
  private IntakeSim intakeSim = null;

  /** Sim-only fuel inside the robot: hopper → spindexer → feeder → flywheel. */
  private FuelPathSim fuelPathSim = null;

  /** Sim-only physics for the flywheel, feeder, spindexer, intake roller, intake wrist, and hood. */
  private MechanismSimManager mechanismSim = null;

  /** Sim-only hopper extension: slides out on first enable and becomes part of the robot's collision. */
  private HopperSim hopperSim = null;

  /** Sim-only Limelights: publish what each camera would see so the real vision code runs. */
  private VisionSim visionSim = null;

  /** Alliance whose spawn the sim robot was last placed at (null until the DS reports one). */
  private Alliance spawnedAlliance = null;

  /** Fuel piece poses for live AdvantageScope field rendering (NT struct arrays). */
  private final StructArrayPublisher<Pose3d> simFuelPoses3dPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("MapleSim/Fuel3d", Pose3d.struct)
          .publish();
  private final StructArrayPublisher<Pose2d> simFuelPoses2dPublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("MapleSim/Fuel2d", Pose2d.struct)
          .publish();
  /** Ground-truth 3D robot pose including bump height/tilt, for the AdvantageScope 3D field. */
  private final StructPublisher<Pose3d> simRobotPose3dPublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("MapleSim/RobotPose3d", Pose3d.struct)
          .publish();

  public Robot() {
    SignalLogger.setPath("/U/ctre-logs/");
    if (RobotBase.isSimulation()) {
      // Override the arena before RobotContainer (which constructs Swerve and registers
      // the drivetrain with the arena). false = no ramp colliders so the robot can drive
      // over the field bumps instead of being stopped by them.
      Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
      // Efficiency mode (MapleSim's default) cuts the neutral zone to a third of its fuel and
      // drops the other alliance's depot. Turn it off for a full match load: 360 neutral + both
      // depots.
      arena.setEfficiencyMode(false);
      SimulatedArena.overrideInstance(arena);
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
    joystickPublisher.update();
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
    hopperSim = new HopperSim(sm.drivetrain.getDriveSimulation());
    mechanismSim = new MechanismSimManager(sm, hopperSim);
    visionSim = new VisionSim();
    // Only collect fuel once the intake arm has physically swung down, not the instant the
    // state changes to INTAKING.
    fuelPathSim = new FuelPathSim();
    intakeSim = new IntakeSim(
        sm.drivetrain.getDriveSimulation(),
        fuelPathSim,
        () -> sm.getCurrentState() == RobotState.INTAKING
            && mechanismSim.isIntakeDeployed()
            && Math.abs(mechanismSim.getIntakeRollerVelocityRps())
                > MechanismSimConstants.INTAKE_ROLLER_INTAKING_RPS);
  }

  @Override
  public void simulationPeriodic() {
    StateMachine sm = m_robotContainer.getStateMachine();

    // Start on our own alliance's side: whenever the alliance changes while disabled (including
    // the first time the DS reports one), move the robot to that alliance's spawn. Never while
    // enabled, so it can't teleport mid-match.
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (DriverStation.isDisabled() && alliance.isPresent() && alliance.get() != spawnedAlliance) {
      sm.drivetrain.resetPose(MechanismSimConstants.spawnPose(alliance.get()));
      spawnedAlliance = alliance.get();
    }

    // Slides the hopper out on first enable and adds/removes its collision box before the arena
    // steps.
    if (hopperSim != null) hopperSim.update();

    // Step mechanism physics first so the intake-deployed check below sees this loop's arm angle.
    if (mechanismSim != null) mechanismSim.update();

    // Toggle the intake zone before the arena ticks so contact detection runs with
    // the correct extended/retracted state for this frame.
    if (intakeSim != null) intakeSim.update();

    // Fuel rolls up and back down the bumps instead of passing through them at floor height.
    FuelBumpSim.update();

    // Advance MapleSim physics. Inside this call, each module's SimulatedMotorController
    // pushes encoder positions into the TalonFX/CANcoder sim states so CTRE's odometry
    // sees fresh sensor values.
    SimulatedArena.getInstance().simulationPeriodic();

    // Bump physics: Z/pitch/roll over the field bumps, and slide back off ones the robot
    // is too slow to crest. Must run before the odometry sync so CTRE sees the corrected X.
    sm.drivetrain.updateBumpSim();
    simRobotPose3dPublisher.set(sm.drivetrain.getSimPose3d());
    SmartDashboard.putBoolean("simOnBump", sm.drivetrain.isSimOnBump());

    // Push MapleSim's gyro reading into the Pigeon2 sim state.
    sm.drivetrain.updateSimState();

    // Limelights see the true pose (with bump tilt); the vision code reads it next loop, like the
    // real cameras' latency.
    if (visionSim != null) visionSim.update(sm.drivetrain.getSimPose3d());

    // With vision fusion off, sync CTRE odometry to MapleSim's ground-truth pose so AdvantageScope
    // shows the robot at the same position where MapleSim runs collision/intake physics. With it
    // on, odometry runs from the simulated wheels and gyro and vision corrects it.
    if (!VisionSim.isFusionEnabled()) sm.drivetrain.syncOdometryFromSim();

    runFuelPath(sm);
    publishSimGamePieces(sm);
  }

  /** Publishes all simulated fuel poses so AdvantageScope can render moving game pieces live. */
  private void publishSimGamePieces(StateMachine sm) {
    Pose3d[] arenaFuel = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
    ArrayList<Pose3d> allFuelPoses = new ArrayList<>(arenaFuel.length + 1);
    for (Pose3d fuel : arenaFuel) {
      // Balls on the floor (not in flight) ride on top of the bumps.
      boolean grounded = fuel.getZ() < MechanismSimConstants.FUEL_RADIUS_M + 0.05;
      allFuelPoses.add(grounded ? FuelBumpSim.onSurface(fuel) : fuel);
    }

    // Fuel inside the robot: flying in from the rollers, piled in the hopper, riding the
    // spindexer, and climbing the feeder.
    if (fuelPathSim != null) allFuelPoses.addAll(fuelPathSim.getPoses(sm.drivetrain.getSimPose3d()));

    Pose3d[] fuel3d = allFuelPoses.toArray(Pose3d[]::new);
    simFuelPoses3dPublisher.set(fuel3d);

    Pose2d[] fuel2d = new Pose2d[fuel3d.length];
    for (int i = 0; i < fuel3d.length; i++) {
      fuel2d[i] = fuel3d[i].toPose2d();
    }
    simFuelPoses2dPublisher.set(fuel2d);

    SmartDashboard.putNumber("simHeldFuelCount", fuelPathSim != null ? fuelPathSim.getCount() : 0);
    SmartDashboard.putNumber("simFuelVisibleCount", fuel3d.length);
  }

  /**
   * Moves fuel through the spindexer and feeder at their simulated speeds, and launches a shot for
   * every ball that reaches the flywheel. The real robot only runs the feeder and spindexer when
   * it's ready to fire, so that gates the shots; the fire rate comes from the mechanisms.
   *
   * <p>Each ball launches from where the simulated flywheel and hood actually ARE (not their
   * setpoints), so shots taken while they're still moving miss like they would for real. See the
   * "Shots" section of {@link MechanismSimConstants} for how the launch model was calibrated.
   */
  private void runFuelPath(StateMachine sm) {
    int shots =
        fuelPathSim.update(
            mechanismSim.getSpindexerVelocityRps(), mechanismSim.getFeederVelocityRps());
    if (shots == 0) return;

    Pose2d pose = sm.drivetrain.getState().Pose;
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(sm.drivetrain.getState().Speeds, pose.getRotation());
    for (int i = 0; i < shots; i++) {
      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              SimShot.launch(
                  pose,
                  fieldRelativeSpeeds,
                  mechanismSim.getFlywheelVelocityRps(),
                  mechanismSim.getHoodPositionRotations()));
    }
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
