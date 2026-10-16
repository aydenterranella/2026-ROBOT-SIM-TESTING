package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeWristConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;
import java.lang.reflect.Field;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.Test;

/**
 * Builds the real {@link Robot}, enables it, and runs the sim loop for a few seconds. Catches
 * wiring mistakes (nulls, wrong component counts, PowerLib still on its kinematic sim) that the
 * per-mechanism tests can't see.
 */
class RobotSimSmokeTest {
  @Test
  void simLoopEndToEnd() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    var componentPoses =
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("Sim/Mechanisms/ComponentPoses", Pose3d.struct)
            .subscribe(new Pose3d[0]);

    Robot robot = SimTestRobots.create();
    robot.robotInit();
    robot.simulationInit();
    // Full match load: 360 in the neutral zone + 24 in each depot.
    assertTrue(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel").length >= 408,
        "field should have a full match load of fuel");

    Field containerField = Robot.class.getDeclaredField("m_robotContainer");
    containerField.setAccessible(true);
    StateMachine sm = ((RobotContainer) containerField.get(robot)).getStateMachine();
    Swerve drive = sm.drivetrain;
    assertTrue(sm.shooter.getVelocityMotor() != null && sm.intakeWrist.getPositionMotor() != null,
        "physics sim should build the real TalonFX devices");

    // Spawns on our alliance's side, and follows alliance changes while disabled.
    run(robot, 0.1);
    assertEquals(2.0, drive.getState().Pose.getX(), 0.05, "blue should spawn on the blue side");
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    run(robot, 0.1);
    assertEquals(16.541 - 2.0, drive.getState().Pose.getX(), 0.05, "red should spawn on the red side");
    assertEquals(180.0, Math.abs(drive.getState().Pose.getRotation().getDegrees()), 1.0,
        "red spawn should face the blue wall");
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    run(robot, 0.1);
    assertEquals(2.0, drive.getState().Pose.getX(), 0.05, "switching back should return to blue");

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();

    run(robot, 2.0);

    Pose3d[] poses = componentPoses.get();
    assertEquals(7, poses.length, "intake, hood, hopper, and 4 wheels");
    // Hopper (index 2) deploys on enable and stops at its measured "out" position.
    assertEquals(MechanismSimConstants.HOPPER_OUT_OFFSET_M, poses[2].getX(), 1e-6,
        "hopper should have slid out");

    // Intake goes down while intaking and back to idle when the request is released.
    Object owner = new Object();
    sm.requestState(RobotState.INTAKING, owner);
    run(robot, 1.5);
    assertEquals(IntakeWristConstants.INTAKE_MAX, sm.intakeWrist.getPositionRotations(), 0.02,
        "intake should be down while intaking");
    sm.clearRequest(owner);
    run(robot, 1.5);
    assertEquals(IntakeWristConstants.INTAKE_IDLE, sm.intakeWrist.getPositionRotations(), 0.02,
        "intake should return to idle after intaking");

    // Hopper holds 65.
    Field fuelField = Robot.class.getDeclaredField("fuelPathSim");
    fuelField.setAccessible(true);
    FuelPathSim fuel = (FuelPathSim) fuelField.get(robot);
    fuel.setCount(100);
    assertEquals(MechanismSimConstants.HOPPER_CAPACITY, fuel.getCount(), "hopper should cap at 65");
    assertEquals(MechanismSimConstants.HOPPER_CAPACITY, fuel.getPoses(new Pose3d()).size(),
        "every held ball should be drawn");

    // Standing still ~2.8 m from the blue hub, holding the shoot trigger with the sticks
    // centered: the robot should turn to aim, spin up, and fire without moving.
    fuel.setCount(10);
    Pose2d start = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90));
    drive.resetPose(start);
    XboxControllerSim pad = new XboxControllerSim(0);
    pad.setRightTriggerAxis(1.0);
    pad.notifyNewData();
    double scoredBefore = fuelInBlueHub();
    run(robot, 4.0);
    assertTrue(fuel.getCount() <= 2,
        "should fire while stationary, " + fuel.getCount() + " of 10 left");
    assertEquals(0.0,
        drive.getState().Pose.getTranslation().getDistance(start.getTranslation()), 0.05,
        "robot should not have moved");
    pad.setRightTriggerAxis(0.0);
    pad.notifyNewData();
    run(robot, 1.5); // let the last shots land
    double fired = 10 - fuel.getCount();
    double scored = fuelInBlueHub() - scoredBefore;
    System.out.println("Stationary shot from 2.8 m: scored " + scored + " of " + fired);
    assertTrue(scored >= fired - 1,
        "the real shot logic should score in sim: " + scored + " of " + fired);
    robot.close();
  }

  private static double fuelInBlueHub() {
    return SimulatedArena.getInstance().blueScoringBreakdown.getOrDefault("TotalFuelInHub", 0.0);
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
