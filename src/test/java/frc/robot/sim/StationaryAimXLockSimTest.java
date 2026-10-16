package frc.robot.sim;

/** SOTM off: the stationary shot X-locks once aimed. */
class StationaryAimXLockSimTest extends StationaryAimSimBase {
  @Override
  boolean sotm() {
    return false;
  }
}
