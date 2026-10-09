package frc.robot.utils;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Publishes every driver station joystick to NetworkTables in the layout AdvantageScope's
 * Joysticks tab reads live ("/AdvantageKit/DriverStation/JoystickN/..."), so controllers show up
 * there without AdvantageKit. Works the same in sim and on the real robot. Call {@link #update()}
 * once per loop.
 */
public class JoystickPublisher {
  private static final int PORTS = DriverStation.kJoystickPorts;

  private final IntegerPublisher[] buttonCount = new IntegerPublisher[PORTS];
  private final IntegerPublisher[] buttonValues = new IntegerPublisher[PORTS];
  private final DoubleArrayPublisher[] axisValues = new DoubleArrayPublisher[PORTS];
  private final IntegerArrayPublisher[] povs = new IntegerArrayPublisher[PORTS];

  public JoystickPublisher() {
    NetworkTable ds = NetworkTableInstance.getDefault().getTable("AdvantageKit/DriverStation");
    for (int port = 0; port < PORTS; port++) {
      NetworkTable table = ds.getSubTable("Joystick" + port);
      buttonCount[port] = table.getIntegerTopic("ButtonCount").publish();
      buttonValues[port] = table.getIntegerTopic("ButtonValues").publish();
      axisValues[port] = table.getDoubleArrayTopic("AxisValues").publish();
      povs[port] = table.getIntegerArrayTopic("POVs").publish();
    }
  }

  public void update() {
    for (int port = 0; port < PORTS; port++) {
      // Buttons are a bitmask: bit i = button i+1 pressed.
      buttonCount[port].set(DriverStation.getStickButtonCount(port));
      buttonValues[port].set(DriverStation.getStickButtons(port));

      double[] axes = new double[DriverStation.getStickAxisCount(port)];
      for (int i = 0; i < axes.length; i++) axes[i] = DriverStation.getStickAxis(port, i);
      axisValues[port].set(axes);

      long[] povValues = new long[DriverStation.getStickPOVCount(port)];
      for (int i = 0; i < povValues.length; i++) povValues[i] = DriverStation.getStickPOV(port, i);
      povs[port].set(povValues);
    }
  }
}
