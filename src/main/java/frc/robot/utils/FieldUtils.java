package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class FieldUtils {
  public static enum GameZone {
    RED_ALLIANCE, // max, max
    NEUTRAL,
    BLUE_ALLIANCE, // 0,0
    INTERCHANGE // not in any specific zone, between alliance and neutral aka going over bump
  }

  public static GameZone getZone(Pose2d pose) {
    double x = pose.getX();

    if (Constants.Field.BLUE_START_X < x && Constants.Field.BLUE_END_X > x) {
      return GameZone.BLUE_ALLIANCE;
    }

    if (Constants.Field.CENTER_START_X < x && Constants.Field.CENTER_END_X > x) {
      return GameZone.NEUTRAL;
    }

    if (Constants.Field.RED_START_X < x && Constants.Field.RED_END_X > x) {
      return GameZone.RED_ALLIANCE;
    }

    return GameZone.INTERCHANGE;
  }

}
