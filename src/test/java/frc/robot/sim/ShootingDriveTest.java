package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.StateMachine;
import java.lang.reflect.Field;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.Test;

/**
 * Drives the real robot code with a simulated Xbox controller and checks how it drives while the
 * shoot trigger is held: slowed down like intaking, and once stopped and aimed, wheels parked
 * straight instead of left angled from the turn (which looked like an X).
 */
class ShootingDriveTest {
  private static void step(Robot robot) throws InterruptedException {
    Unmanaged.feedEnable(100);
    DriverStationSim.notifyNewData();
    robot.robotPeriodic();
    robot.teleopPeriodic();
    robot.simulationPeriodic();
    Thread.sleep(20);
  }

  private static double speed(StateMachine sm) {
    var speeds = sm.drivetrain.getState().Speeds;
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  @Test
  void slowsDownAndParksWheelsStraightWhileShooting() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    Robot robot = new Robot();
    robot.robotInit();
    robot.simulationInit();
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();
    Field container = Robot.class.getDeclaredField("m_robotContainer");
    container.setAccessible(true);
    StateMachine sm = ((RobotContainer) container.get(robot)).getStateMachine();
    XboxControllerSim pad = new XboxControllerSim(0);
    for (int i = 0; i < 10; i++) step(robot);

    // Full stick, no trigger, for 1 s.
    sm.drivetrain.resetPose(new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(90)));
    pad.setLeftY(-1.0);
    pad.notifyNewData();
    double normalTop = 0;
    for (int i = 0; i < 50; i++) {
      step(robot);
      normalTop = Math.max(normalTop, speed(sm));
    }

    // Same stick with the shoot trigger: should settle around 0.4x.
    pad.setRightTriggerAxis(1.0);
    pad.notifyNewData();
    double shootingTop = 0;
    for (int i = 0; i < 50; i++) {
      step(robot);
      if (i >= 25) shootingTop = Math.max(shootingTop, speed(sm)); // after it slows down
    }
    assertTrue(shootingTop < 0.6 * normalTop,
        String.format("shooting should be slower: %.2f vs %.2f m/s", shootingTop, normalTop));

    // Let go of the stick, keep shooting: once aimed, every wheel should point straight
    // (0° or 180°), not stay angled from turning in place.
    pad.setLeftY(0.0);
    pad.notifyNewData();
    for (int i = 0; i < 75; i++) step(robot);
    var targets = sm.drivetrain.getState().ModuleTargets;
    for (int m = 0; m < 4; m++) {
      double deg = Math.abs(targets[m].angle.getDegrees());
      double offStraight = Math.min(deg, 180.0 - deg);
      assertTrue(offStraight < 1.0,
          "wheel " + m + " should be parked straight, was " + targets[m].angle.getDegrees() + "°");
    }
    assertTrue(sm.isReadyToFire(), "should still be aimed and ready to fire");

    // A partial stick push (60%) while shooting should actually drive the robot, with every wheel
    // pointed the way it's going. (The old aim mode's deadband ate everything under ~73% stick,
    // so the robot just sat and turned, leaving the wheels in an X.) And it should still score.
    sm.drivetrain.resetPose(new Pose2d(1.0, 5.5, Rotation2d.kZero));
    Field fuelField = Robot.class.getDeclaredField("fuelPathSim");
    fuelField.setAccessible(true);
    FuelPathSim fuel = (FuelPathSim) fuelField.get(robot);
    fuel.setCount(12);
    double scoredBefore = fuelInBlueHub();
    pad.setLeftY(-0.6);
    pad.notifyNewData();
    for (int i = 0; i < 50; i++) step(robot); // turn to aim, settle
    double movingSpeed = 0;
    for (int i = 0; i < 50; i++) {
      step(robot);
      movingSpeed = Math.max(movingSpeed, speed(sm));
    }
    assertTrue(movingSpeed > 0.1, "should drive while shooting, top speed " + movingSpeed);
    var moving = sm.drivetrain.getState().ModuleTargets;
    for (int m = 1; m < 4; m++) {
      double diff = Math.abs(moving[m].angle.minus(moving[0].angle).getDegrees());
      double offLine = Math.min(diff, 180.0 - diff); // a wheel spun backward is still in line
      assertTrue(offLine < 10.0,
          "wheels should all point the way it's driving, wheel " + m + " is off by " + offLine + "°");
    }
    int fired = 12 - fuel.getCount();
    pad.setLeftY(0.0);
    pad.setRightTriggerAxis(0.0);
    pad.notifyNewData();
    for (int i = 0; i < 75; i++) step(robot); // let the last shots land
    double scored = fuelInBlueHub() - scoredBefore;
    System.out.println("moving shots: fired " + fired + ", scored " + scored);
    assertTrue(fired >= 6, "should keep firing while moving, fired " + fired);
    assertTrue(scored >= fired - 1, "should score while moving: " + scored + " of " + fired);
    robot.close();
  }

  private static double fuelInBlueHub() {
    return SimulatedArena.getInstance().blueScoringBreakdown.getOrDefault("TotalFuelInHub", 0.0);
  }
}
