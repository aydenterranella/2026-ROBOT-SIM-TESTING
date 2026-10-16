package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.powerlib.auto.AutoBuilder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.autos.RobotAutos;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Locale;
import org.ironmaple.simulation.SimulatedArena;
import org.junit.jupiter.api.Test;

/**
 * Runs one autonomous route end to end on the real robot code in sim: start pose, chooser
 * selection, the full 15 s, intake through the neutral zone, the return and the shot. Fails if
 * the auto aborts early, misses a waypoint, collects nothing or scores nothing.
 */
abstract class AutoRouteSimBase {
  /** The game allows up to 8 preloaded fuel. */
  private static final int PRELOAD = 8;

  private long nextTick;

  abstract String routeName();

  @Test
  void runsTheFullAutoAndScores() throws Exception {
    RobotAutos.Route route = RobotAutos.ROUTES.stream()
        .filter(r -> r.name().equals(routeName())).findFirst().orElseThrow();
    boolean red = route.alliance() == Alliance.Red;
    List<Pose2d> points = route.fieldPoints();

    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    Robot robot = SimTestRobots.create();
    robot.robotInit();
    robot.simulationInit();
    Field containerField = Robot.class.getDeclaredField("m_robotContainer");
    containerField.setAccessible(true);
    StateMachine stateMachine = ((RobotContainer) containerField.get(robot)).getStateMachine();
    Swerve drive = stateMachine.drivetrain;
    Field fuelField = Robot.class.getDeclaredField("fuelPathSim");
    fuelField.setAccessible(true);
    FuelPathSim fuel = (FuelPathSim) fuelField.get(robot);

    // Disabled: spawn on our side, then place the robot on the route's start like a driver would.
    nextTick = System.nanoTime();
    run(robot, 0.5);
    drive.resetPose(points.get(0));
    fuel.setCount(PRELOAD);
    NetworkTableInstance.getDefault().getTable("SmartDashboard/" + AutoBuilder.DASHBOARD_KEY)
        .getEntry("selected").setString(route.name());
    run(robot, 0.5);

    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.autonomousInit();
    Field autoField = Robot.class.getDeclaredField("m_autonomousCommand");
    autoField.setAccessible(true);
    Command auto = (Command) autoField.get(robot);
    assertTrue(auto != null && auto.getName().equals(route.name()),
        "chooser should hand out " + route.name() + ", got " + (auto == null ? null : auto.getName()));

    double[] closest = new double[points.size()];
    double[] reachedAt = new double[points.size()];
    java.util.Arrays.fill(closest, Double.POSITIVE_INFINITY);
    java.util.Arrays.fill(reachedAt, Double.NaN);
    double scoredBefore = scored(red);
    double endedAt = Double.NaN, firstShotAt = Double.NaN;
    int mostFuel = fuel.getCount(), lastFuel = fuel.getCount();
    var lastState = stateMachine.getCurrentState();
    StringBuilder log = new StringBuilder(), trace = new StringBuilder();
    int ticks = (int) Math.round((Constants.Field.AUTO_LENGTH_IN_TIME + 0.5) / 0.02);
    for (int k = 1; k <= ticks; k++) {
      run(robot, 0.02);
      double t = k * 0.02;
      Pose2d pose = drive.getDriveSimulation().getSimulatedDriveTrainPose();
      for (int i = 0; i < points.size(); i++) {
        double d = pose.getTranslation().getDistance(points.get(i).getTranslation());
        if (d < closest[i]) closest[i] = d;
        if (Double.isNaN(reachedAt[i]) && d < 0.30) reachedAt[i] = t;
      }
      int count = fuel.getCount();
      mostFuel = Math.max(mostFuel, count);
      if (count < lastFuel && Double.isNaN(firstShotAt)) firstShotAt = t;
      lastFuel = count;
      var state = stateMachine.getCurrentState();
      if (k % 5 == 0) {
        var speeds = drive.getDriveSimulation().getDriveTrainSimulatedChassisSpeedsRobotRelative();
        trace.append(String.format(Locale.ROOT, "  %5.2fs (%.2f, %.2f) %.2f m/s %s%n", t, pose.getX(), pose.getY(),
            Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), state));
      }
      if (state != lastState) {
        log.append(String.format(Locale.ROOT, "  %5.2fs %s -> %s at (%.2f, %.2f)%n", t, lastState, state,
            pose.getX(), pose.getY()));
        lastState = state;
      }
      if (Double.isNaN(endedAt) && !auto.isScheduled()) endedAt = t;
    }
    Pose2d end = drive.getDriveSimulation().getSimulatedDriveTrainPose();
    double scored = scored(red) - scoredBefore;

    StringBuilder report = new StringBuilder(String.format(Locale.ROOT,
        "%s: ended %s, fuel preload %d -> most %d -> left %d, scored %.0f, first shot %.2fs, end (%.2f, %.2f)%n",
        route.name(), Double.isNaN(endedAt) ? "after the auto period" : String.format(Locale.ROOT, "at %.2fs", endedAt),
        PRELOAD, mostFuel, fuel.getCount(), scored, firstShotAt, end.getX(), end.getY()));
    for (int i = 0; i < points.size(); i++) {
      report.append(String.format(Locale.ROOT, "  waypoint %d (%.2f, %.2f): closest %.3f m, within 0.30 m at %s%n",
          i + 1, points.get(i).getX(), points.get(i).getY(), closest[i],
          Double.isNaN(reachedAt[i]) ? "never" : String.format(Locale.ROOT, "%.2fs", reachedAt[i])));
    }
    report.append(log);
    System.out.print(report);
    System.out.print("  trace every 0.1 s:\n" + trace);

    // A finished auto holds its aim until the period runs out; ending earlier means it aborted.
    assertTrue(Double.isNaN(endedAt) || endedAt >= Constants.Field.AUTO_LENGTH_IN_TIME - 0.1,
        "auto should run the whole period, not abort\n" + report);
    for (int i = 0; i < points.size(); i++) {
      double tolerance = edu.wpi.first.math.util.Units.inchesToMeters(Constants.Auto.LEG_TOLERANCE_INCHES[i]);
      assertTrue(closest[i] <= Math.max(tolerance, 0.15) + 0.10,
          "should pass waypoint " + (i + 1) + ", closest " + closest[i] + " m\n" + report);
    }
    assertTrue(mostFuel > PRELOAD, "should collect fuel in the neutral zone\n" + report);
    assertTrue(scored >= PRELOAD, "should score at least the preload\n" + report);
    robot.close();
  }

  private static double scored(boolean red) {
    var arena = SimulatedArena.getInstance();
    return (red ? arena.redScoringBreakdown : arena.blueScoringBreakdown).getOrDefault("TotalFuelInHub", 0.0);
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
