package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.powerlib.auto.AutoBuilder;
import frc.robot.Robot;
import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The Sim GUI starts "Disconnected" with no alliance, which left the alliance-filtered auto
 * chooser offering only "None" until the robot had been enabled once. The sim now defaults to
 * Blue 1, so a route can be picked while disabled, before the first enable.
 */
class ChooserBeforeEnableSimTest {
  @Test
  void routeCanBePickedBeforeTheFirstEnable() throws Exception {
    assertTrue(HAL.initialize(500, 0)); // no alliance set, like the Sim GUI at launch
    Robot robot = SimTestRobots.create();
    robot.robotInit();
    robot.simulationInit();
    NetworkTable chooser = NetworkTableInstance.getDefault().getTable("SmartDashboard/" + AutoBuilder.DASHBOARD_KEY);
    step(robot, 10);

    List<String> options = List.of(chooser.getEntry("options").getStringArray(new String[0]));
    assertTrue(options.contains("Blue Left") && options.contains("Blue Right"),
        "routes should be offered before any enable, got " + options);

    chooser.getEntry("selected").setString("Blue Left");
    step(robot, 5);
    assertEquals("Blue Left", chooser.getEntry("active").getString(""), "selection should stick while disabled");

    // Picking Autonomous in the GUI attaches the DS with the station from its Alliance box.
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(robot, 1);
    robot.autonomousInit();
    Field autoField = Robot.class.getDeclaredField("m_autonomousCommand");
    autoField.setAccessible(true);
    Command auto = (Command) autoField.get(robot);
    assertEquals("Blue Left", auto == null ? null : auto.getName(), "Autonomous should run the picked route");
    robot.close();
  }

  private static void step(Robot robot, int loops) throws InterruptedException {
    for (int i = 0; i < loops; i++) {
      Unmanaged.feedEnable(100);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      robot.simulationPeriodic();
      Thread.sleep(20);
    }
  }
}
