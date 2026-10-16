package frc.robot.sim;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;

/** Builds the real {@link Robot} for headless sim tests. */
final class SimTestRobots {
  private SimTestRobots() {}

  /**
   * {@code RobotBase} starts a NetworkTables server on the robot's ports (5810/1735). A test JVM
   * holding them would take Power Tool's and AdvantageScope's connection away from a sim you are
   * running, so the test robot runs NetworkTables locally instead. Same entries, no sockets.
   */
  static Robot create() {
    Robot robot = new Robot();
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.stopServer();
    nt.startLocal();
    return robot;
  }
}
