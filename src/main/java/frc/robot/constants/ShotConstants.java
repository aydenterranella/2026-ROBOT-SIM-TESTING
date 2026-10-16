package frc.robot.constants;

import frc.firecontrol.ShotLUT;
import frc.firecontrol.ShotParameters;
import frc.lib.team9410.math.LinearInterpolator;

public class ShotConstants {

    // Shooter Tables 
    public static final double[][] SHOOTER_SPEEDS = {
        { 1.68, 62 },
        { 1.97, 64 },
        { 2.38, 67 },
        { 2.75, 71 },
        { 3.29, 74 },
        { 3.75, 77 },
        { 4.05, 79 },
        { 4.60, 82 },
        { 4.85, 84 },
        { 5.50, 89 },
    };

    public static final double[][] HOOD_ANGLES = {
        { 1.68, 0.020 },
        { 1.97, 0.030 },
        { 2.38, 0.040 },
        { 2.75, 0.050 },
        { 3.29, 0.060 },
        { 3.75, 0.070 },
        { 4.05, 0.080 },
        { 4.60, 0.090 },
        { 4.85, 0.105 },
        { 5.50, 0.135 },
    };

      public static final double[][] FEEDER_SPEEDS = {
        { 1.68, 58 },
        { 1.97, 59 },
        { 2.38, 61 },
        { 2.75, 63 },
        { 3.29, 65 },
        { 3.75, 65 },
        { 4.05, 66 },
        { 4.60, 67 },
        { 4.85, 68 },
        { 5.50, 72 },
    };
    
    // TOF interpolated
    public static final double[][] SHOOTER_TOF = {
        { 1.68, 0.515 },
        { 1.97, 0.574 },
        { 2.38, 0.658 },
        { 2.75, 0.733 },
        { 3.29, 0.812 },
        { 3.75, 0.856 },
        { 4.05, 0.884 },
        { 4.60, 0.947 },
        { 4.85, 0.976 },
        { 5.50, 1.052 },
    };

  

    // Interpolators
    public static final LinearInterpolator SHOOTER_VELOCITY_INTERPOLATOR =
        new LinearInterpolator(SHOOTER_SPEEDS);

    public static final LinearInterpolator HOOD_ANGLE_INTERPOLATOR =
        new LinearInterpolator(HOOD_ANGLES);

    public static final LinearInterpolator FEEDER_VELOCITY_INTERPOLATOR =
        new LinearInterpolator(FEEDER_SPEEDS);

    // SOTM fire-control config
    // Shooter offset from robot center along X (negative = rear-mounted).
    public static final double LAUNCHER_OFFSET_X = -0.20;

    // Shooter faces the rear of the robot.
    public static final double SHOOTER_ANGLE_OFFSET_RAD = Math.PI;

    // Max robot speed (m/s) for SOTM solver; above this returns INVALID.
    public static final double MAX_SOTM_SPEED = 3.0;

    // Min/max scoring distance (m).
    public static final double MIN_SCORING_DISTANCE = 0.5;
    public static final double MAX_SCORING_DISTANCE = FieldConstants.MAX_DISTANCE_TO_HOPPER;

    // Robot heading must be within this many degrees of aim angle to fire.
    public static final double HEADING_TOLERANCE_DEG = 10.0;

    // Combined ShotLUT
    public static final ShotLUT SHOT_LUT;
    static {
        SHOT_LUT = new ShotLUT();
        for (int i = 0; i < SHOOTER_SPEEDS.length; i++) {
            double dist     = SHOOTER_SPEEDS[i][0];
            double rps      = SHOOTER_SPEEDS[i][1];
            double hoodDeg  = HOOD_ANGLES[i][1] * 360.0; // rotations → degrees for ShotParameters
            double tof      = SHOOTER_TOF[i][1];
            SHOT_LUT.put(dist, new ShotParameters(rps, hoodDeg, tof));
        }
    }
}
