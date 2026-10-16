package frc.robot.sim;

import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import java.lang.reflect.Field;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.junit.jupiter.api.Test;

/**
 * Steers the sim wheels on the robot's real steer gains, ratio and fused CANcoder (only offsets
 * and inversions are zeroed for sim) and checks they turn, settle and hold without buzzing. A
 * too-heavy MapleSim steer model makes the 85° step miss the settle time.
 */
class SwerveSteerSimTest {
  private long nextTick;

  @Test
  void wheelsSettleAndHoldOnRealSteerValues() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    Robot robot = SimTestRobots.create();
    robot.robotInit();
    robot.simulationInit();
    Field containerField = Robot.class.getDeclaredField("m_robotContainer");
    containerField.setAccessible(true);
    Swerve drive = ((RobotContainer) containerField.get(robot)).getStateMachine().drivetrain;
    SwerveModuleSimulation[] modules = drive.getDriveSimulation().getModules();

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();
    // Command the modules directly instead of through the driver command.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().removeDefaultCommand(drive);
    nextTick = System.nanoTime();

    // Phoenix's sim firmware runs on the wall clock, so warm the JIT up with one step cycle
    // before timing anything.
    var point = new SwerveRequest.PointWheelsAt();
    drive.setControl(point.withModuleDirection(Rotation2d.fromDegrees(85)));
    run(robot, 0.6);
    drive.setControl(point.withModuleDirection(Rotation2d.kZero));
    run(robot, 0.6);

    // 85° step: big enough to run into the 60 A steer current limit. ~0.3 s in sim today.
    drive.setControl(point.withModuleDirection(Rotation2d.fromDegrees(85)));
    run(robot, 0.5);
    for (var module : modules) {
      assertTrue(Math.abs(error(module, 85)) < 2,
          "wheel should settle within 0.5 s, off by " + error(module, 85) + "°");
    }
    double worstError = 0, worstVolts = 0;
    for (int i = 0; i < 50; i++) {
      run(robot, 0.02);
      for (var module : modules) {
        worstError = Math.max(worstError, Math.abs(error(module, 85)));
        worstVolts = Math.max(worstVolts, Math.abs(module.getSteerMotorAppliedVoltage().in(Volts)));
      }
    }
    assertTrue(worstError < 0.5, "wheels should hold still, wandered " + worstError + "°");
    assertTrue(worstVolts < 0.5, "steer motors should not buzz at rest, saw " + worstVolts + " V");

    // X-lock (what a stationary shot parks in): every wheel reaches its own target.
    drive.setControl(new SwerveRequest.SwerveDriveBrake());
    run(robot, 0.5);
    var targets = drive.getState().ModuleTargets;
    for (int i = 0; i < modules.length; i++) {
      double target = targets[i].angle.getDegrees();
      assertTrue(Math.abs(error(modules[i], target)) < 2,
          "module " + i + " should reach X-lock, off by " + error(modules[i], target) + "°");
    }
    robot.close();
  }

  /** Steer error in degrees, folded to ±90° since a reversed wheel is the same heading. */
  private static double error(SwerveModuleSimulation module, double targetDegrees) {
    return MathUtil.inputModulus(module.getSteerAbsoluteFacing().getDegrees() - targetDegrees, -90, 90);
  }

  /** Steps the robot and MapleSim on a 20 ms wall-clock cadence, like TimedRobot. */
  private void run(Robot robot, double seconds) throws InterruptedException {
    for (int i = 0; i < (int) Math.round(seconds / 0.02); i++) {
      Unmanaged.feedEnable(100);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      robot.simulationPeriodic();
      nextTick += 20_000_000L;
      long sleepNanos = nextTick - System.nanoTime();
      if (sleepNanos > 0) {
        Thread.sleep(sleepNanos / 1_000_000, (int) (sleepNanos % 1_000_000));
      } else {
        nextTick = System.nanoTime();
      }
    }
  }
}
