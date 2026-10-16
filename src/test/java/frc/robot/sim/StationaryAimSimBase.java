package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import frc.powerlib.PowerRobotContainer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.ShootingUtils;
import java.lang.reflect.Field;
import java.util.Locale;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.junit.jupiter.api.Test;

/**
 * Holds shoot with the sticks centered and checks the wheels park once and stay parked. When the
 * sim still applied the robot's steer-to-drive coupling compensation (MapleSim has no coupling),
 * each steer move read as ~0.5 m/s of wheel speed, so the stationary X-lock let go, re-aimed and
 * locked again every ~0.15 s, spinning the wheels over 1,300° while the robot sat still.
 */
abstract class StationaryAimSimBase {
  /** Turns ~150° to aim from 4.2 m, so the robot starts in the turn-in-place stance. */
  private static final Pose2d START = new Pose2d(1.2, 6.5, Rotation2d.fromDegrees(-60));

  private long nextTick;

  abstract boolean sotm();

  @Test
  void parksOnceAndHoldsWhileShooting() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    Robot robot = SimTestRobots.create();
    robot.robotInit();
    robot.simulationInit();
    Field containerField = Robot.class.getDeclaredField("m_robotContainer");
    containerField.setAccessible(true);
    StateMachine stateMachine = ((RobotContainer) containerField.get(robot)).getStateMachine();
    Swerve drive = stateMachine.drivetrain;
    SwerveModuleSimulation[] modules = drive.getDriveSimulation().getModules();
    Field fuelField = Robot.class.getDeclaredField("fuelPathSim");
    fuelField.setAccessible(true);
    FuelPathSim fuel = (FuelPathSim) fuelField.get(robot);
    ShootingUtils.setSotmEnabled(sotm());

    nextTick = System.nanoTime();
    run(robot, 0.3);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();
    run(robot, 1.0);
    drive.resetPose(START);
    fuel.setCount(40);
    XboxControllerSim pad = new XboxControllerSim(0);
    pad.setRightTriggerAxis(1.0);
    pad.notifyNewData();

    // Aim: up to 3 s to park the first time. Turning in place is not driving, so the aim target
    // should hold still while the robot swings round to it.
    int ticks = 0;
    double firstTarget = Double.NaN, targetSwing = 0;
    while (!parked() && ticks++ < 150) {
      run(robot, 0.02);
      Double target = drive.getHeadingSetpointDegrees();
      if (target == null) continue;
      if (Double.isNaN(firstTarget)) firstTarget = target;
      targetSwing = Math.max(targetSwing, Math.abs(MathUtil.inputModulus(target - firstTarget, -180, 180)));
    }
    assertTrue(parked(), "robot should aim and park within 3 s, state " + stateMachine.getCurrentState());
    assertTrue(targetSwing < 1, "aim target should not swing while turning in place, moved " + targetSwing + "°");

    // Let the park move finish, then hold for 4 s.
    run(robot, 0.6);
    Translation2d heldAt = drive.getDriveSimulation().getSimulatedDriveTrainPose().getTranslation();
    double[] heldSteer = steerDegrees(modules);
    int fuelAtPark = fuel.getCount();
    int unparks = 0;
    double worstSteer = 0, worstHeadingError = 0;
    boolean wasParked = true;
    for (int i = 0; i < 200; i++) {
      run(robot, 0.02);
      boolean nowParked = parked();
      if (wasParked && !nowParked) unparks++;
      wasParked = nowParked;
      double[] steer = steerDegrees(modules);
      for (int m = 0; m < steer.length; m++) {
        worstSteer = Math.max(worstSteer, Math.abs(steer[m] - heldSteer[m]));
      }
      Double target = drive.getHeadingSetpointDegrees();
      if (target != null) {
        double heading = drive.getState().Pose.getRotation().getDegrees();
        worstHeadingError = Math.max(worstHeadingError,
            Math.abs(MathUtil.inputModulus(target - heading, -180, 180)));
      }
    }
    double slid = drive.getDriveSimulation().getSimulatedDriveTrainPose().getTranslation()
        .getDistance(heldAt);
    int fired = fuelAtPark - fuel.getCount();
    System.out.printf(Locale.ROOT,
        "sotm=%s locked=%s aimSwing=%.2f° unparks=%d steerWander=%.2f° slid=%.4f m headingErr=%.2f° fired=%d%n",
        sotm(), PowerRobotContainer.getData("Drive/AimWheelsLocked"), targetSwing, unparks, worstSteer, slid,
        worstHeadingError, fired);

    assertTrue(unparks == 0, "wheels should stay parked while shooting, unparked " + unparks + " times");
    assertTrue(worstSteer < 2, "parked wheels should hold still, wandered " + worstSteer + "°");
    assertTrue(slid < 0.01, "robot should not slide while parked, moved " + slid + " m");
    assertTrue(worstHeadingError < 2, "heading should stay on target, off by " + worstHeadingError + "°");
    assertTrue(Boolean.valueOf(!sotm()).equals(PowerRobotContainer.getData("Drive/AimWheelsLocked")),
        "SOTM off should X-lock, SOTM on should park parallel");
    assertTrue(fired >= 10, "should keep shooting while parked, fired " + fired);
    robot.close();
  }

  private static boolean parked() {
    return Boolean.TRUE.equals(PowerRobotContainer.getData("Drive/AimWheelsParked"));
  }

  /** Unwrapped steer angles, so a wheel that spins a full turn shows up as 360°. */
  private static double[] steerDegrees(SwerveModuleSimulation[] modules) {
    double[] degrees = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      degrees[i] = Math.toDegrees(modules[i].getSteerAbsoluteAngle().in(edu.wpi.first.units.Units.Radians));
    }
    return degrees;
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
