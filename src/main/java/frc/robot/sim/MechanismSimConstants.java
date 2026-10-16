// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SpindexerConstants;

/**
 * Physical numbers for the mechanism simulations, pulled from the Onshape CAD and the SysId
 * constants already in the code. Sim-only — nothing on the real robot reads this file.
 *
 * <h2>Coordinate frame</h2>
 * Everything here is in the WPILib robot frame: +X forward (intake side), +Y left, +Z up, with the
 * origin at the center of the robot on the floor. The shooter faces the REAR of the robot.
 *
 * <p>The CAD Main Assembly uses a different frame. To convert a CAD point (inches):
 * <pre>
 *   x_robot = -Y_cad
 *   y_robot =  X_cad - 17.047      (17.047" is the robot centerline in CAD)
 *   z_robot =  Z_cad               (CAD Z = 0 is roughly the floor)
 * </pre>
 *
 * <h2>Where the numbers came from</h2>
 * <ul>
 *   <li><b>CAD</b> — Onshape "Mass and section properties" on individual parts (centroid + MOI).
 *   <li><b>SysId</b> — rotor inertia is back-calculated from the kA values in the *Constants files
 *       (see {@link #inertiaFromKa}). This is usually better than estimating from CAD because it
 *       includes belts, pulleys, and bearings.
 *   <li><b>Estimate</b> — anything marked ESTIMATE is a guess; tune it if the sim looks wrong.
 * </ul>
 */
public final class MechanismSimConstants {
  private MechanismSimConstants() {}

  /** Robot periodic loop time. All mechanism sims step once per robot loop. */
  public static final double LOOP_PERIOD_SECS = 0.02;
  // ---------------------------------------------------------------------------------------------
  // Spawn. The REBUILT field is 180° rotationally symmetric, so red's spawn is blue's spun about
  // the field center (MapleSim field: 16.541 x 8.052 m).
  // ---------------------------------------------------------------------------------------------
  private static final double FIELD_LENGTH_M = 16.541;
  private static final double FIELD_WIDTH_M = 8.052;

  /** Blue spawn: inside the blue alliance zone, facing the red wall. */
  public static final Pose2d BLUE_SPAWN_POSE = new Pose2d(2.0, 4.0, Rotation2d.kZero);

  /** Where the sim robot starts for the given alliance. */
  public static Pose2d spawnPose(Alliance alliance) {
    if (alliance == Alliance.Blue) return BLUE_SPAWN_POSE;
    return new Pose2d(
        FIELD_LENGTH_M - BLUE_SPAWN_POSE.getX(),
        FIELD_WIDTH_M - BLUE_SPAWN_POSE.getY(),
        BLUE_SPAWN_POSE.getRotation().plus(Rotation2d.k180deg));
  }

  /**
   * Back-calculates the rotor-side moment of inertia (kg·m²) from a SysId kA value.
   *
   * <p>SysId kA is "volts per (rotation/s²)". For a DC motor, voltage needed to accelerate is
   * V = (J · α · R) / Kt, so J = kA · Kt / (2π · R). {@code gearbox} must be the combined
   * DCMotor (e.g. {@code getKrakenX60Foc(3)}) so R already accounts for multiple motors.
   */
  public static double inertiaFromKa(double kA, DCMotor gearbox) {
    return kA * gearbox.KtNMPerAmp / (2.0 * Math.PI * gearbox.rOhms);
  }

  // ---------------------------------------------------------------------------------------------
  // Shooter flywheel — 3x Kraken X60 (leader + 2 followers), direct drive, 4" wheels.
  // kV from SysId (0.124 V/rps) matches a bare X60 FOC free speed, so the ratio really is 1:1.
  // Velocity control uses FOC, so we model the FOC motor curve.
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(3);
  /** SysId-derived, ~1.15e-3 kg·m² at the motor shaft. */
  public static final double FLYWHEEL_J_KG_M2 =
      inertiaFromKa(ShooterConstants.FLYWHEEL_KA, FLYWHEEL_GEARBOX);
  /** Friction = SysId kS (volts). */
  public static final double FLYWHEEL_FRICTION_VOLTS = ShooterConstants.FLYWHEEL_KS;

  // ---------------------------------------------------------------------------------------------
  // Feeder — 2x motors, 1:1 in code. SysId kV (0.098 V/rps) is too low for an X60
  // (would be ~0.12), but matches a Kraken X44 — so we model X44s. Confirm on the robot.
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor FEEDER_GEARBOX = DCMotor.getKrakenX44Foc(2);
  public static final double FEEDER_J_KG_M2 =
      inertiaFromKa(FeederConstants.FEEDER_KA, FEEDER_GEARBOX);
  public static final double FEEDER_FRICTION_VOLTS = FeederConstants.FEEDER_KS;

  // ---------------------------------------------------------------------------------------------
  // Spindexer — 1x Kraken X60, 1:1 in code (kV 0.124 matches X60).
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor SPINDEXER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double SPINDEXER_J_KG_M2 =
      inertiaFromKa(SpindexerConstants.SPINDEXER_KA, SPINDEXER_GEARBOX);
  public static final double SPINDEXER_FRICTION_VOLTS = SpindexerConstants.SPINDEXER_KS;

  // ---------------------------------------------------------------------------------------------
  // Intake roller — 1 motor, torque-current FOC. No SysId kA exists for this one.
  // ESTIMATE: two ~2" OD roller tubes at 0.64 lb each (CAD) ≈ 1.7e-4 kg·m² each, plus pulleys.
  // Motor type ESTIMATE: X60 (a Kraken X60 sits near the intake in the CAD tree).
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor INTAKE_ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final double INTAKE_ROLLER_J_KG_M2 = 4.0e-4;
  /** ESTIMATE: no SysId kS for the roller; ~0.3 V is typical of the other rollers. */
  public static final double INTAKE_ROLLER_FRICTION_VOLTS = 0.3;

  // ---------------------------------------------------------------------------------------------
  // Intake wrist (pivot arm) — 2x Kraken X44, 44.44:1 (12→60, 18→60, 12→32 #25 sprocket, which
  // matches WRIST_ROTOR_TO_SENSOR_RATIO). CANcoder is on the pivot axis.
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor INTAKE_WRIST_GEARBOX = DCMotor.getKrakenX44(2);

  /** Pivot axle (26" hex shaft). CAD: Y=-12.866", Z=9.101". */
  public static final Translation3d INTAKE_PIVOT =
      new Translation3d(Units.inchesToMeters(12.866), 0.0, Units.inchesToMeters(9.101));

  /** Pivot → lower (front) roller centerline. CAD: 12.11". */
  public static final double INTAKE_ARM_LENGTH_M = Units.inchesToMeters(12.11);

  /**
   * ESTIMATE: arm MOI about the pivot. Built from CAD parts: two polycarb side plates
   * (0.77 lb each), two rollers (0.64 lb each), shafts/pulleys. ~1.8 kg total.
   */
  public static final double INTAKE_WRIST_J_KG_M2 = 0.09;

  /**
   * Reference pose for converting code units ↔ physical angle. INTAKE_MAX is the "down" setpoint,
   * where the real intake rides 1-2" off the floor. The CAD draws the arm ~2° below horizontal
   * with its lowest point (the bottom scoop) 2.3" up; another 5.5° down puts that at 1.5".
   * Physical angle convention: 0° = arm pointing straight forward, + = rotating up/back.
   */
  public static final double INTAKE_REF_POSITION_ROT = IntakeConstants.INTAKE_MAX;
  public static final double INTAKE_REF_ANGLE_RAD = Math.toRadians(-2.0 - 5.5);
  /** Angle of the arm as drawn in the CAD (used to rotate exported CAD models). */
  public static final double INTAKE_CAD_ANGLE_RAD = Math.toRadians(-2.0);

  /** CANcoder sits on the pivot, so 1 mechanism rotation = 1 arm rotation. */
  public static final double INTAKE_ARM_RAD_PER_MECH_RAD = 1.0;

  /** Hard stops, expressed in code units (mechanism rotations) with a little margin. */
  public static final double INTAKE_HARD_STOP_DOWN_ROT = IntakeConstants.INTAKE_MAX - 0.01;
  public static final double INTAKE_HARD_STOP_UP_ROT = IntakeConstants.INTAKE_MIN + 0.01;

  /** Robots start a match stowed, so the sim starts with the intake up. */
  public static final double INTAKE_START_ROT = IntakeConstants.INTAKE_MIN;

  // ---------------------------------------------------------------------------------------------
  // Shooter hood — the INNER body of the shooter: two narrow truss links that ride on the 4"
  // flywheel's axle and carry the two upper roller shafts and two 190t (10DP) gear sectors. The
  // sectors' tooth tips are 9.6" from the flywheel axle, so the whole inner body swings about it.
  // The outer side plates, flywheels, and motors are bolted to the baseplate and DON'T move; the
  // hood's Kraken X44 sits on the outer plates and its 10t pinion drives the sectors.
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX44(1);

  /** Pivot = flywheel axle (11" hex shaft). CAD (robot frame): x=-0.273, z=0.517 m. */
  public static final Translation3d HOOD_PIVOT = new Translation3d(-0.272, 0.0, 0.517);

  /** Pivot → upper roller shaft at the front end of the links. CAD: 8.25". */
  public static final double HOOD_ARM_LENGTH_M = 0.21;

  /**
   * ESTIMATE: hood MOI about the pivot. Most shooter parts have no material in CAD; ~1.2 kg (links,
   * sectors, two roller shafts) with its CG ~0.18 m out.
   */
  public static final double HOOD_J_KG_M2 = 0.04;

  /**
   * The CAD draws the hood at the BOTTOM of its travel: the spacer plates between the sectors sit
   * on the baseplate, and the pinion is near the top end of the sector teeth so the hood can only
   * swing up from there. The links point 22° above horizontal toward the robot front in that pose.
   *
   * <p>SHOOTER_HOOD_DEFAULT (0.0) is where the hood rests, so code position 0.0 = the CAD pose.
   * Increasing code position raises the front of the hood. Physical angle convention: 0° = links
   * pointing straight forward, + = front end rising.
   */
  public static final double HOOD_CAD_ANGLE_RAD = Math.toRadians(22.1);
  public static final double HOOD_REF_POSITION_ROT = ShooterConstants.SHOOTER_HOOD_DEFAULT;
  public static final double HOOD_REF_ANGLE_RAD = HOOD_CAD_ANGLE_RAD;

  /**
   * How many radians the arm moves per radian of "hood position" in code.
   *
   * <p>The hood CANcoder is NOT on the pivot: in CAD a 46t gear on the arm drives a 24t gear on
   * the encoder shaft, so the CANcoder turns 46/24 = 1.92x the arm. The code's hood range
   * (0.005–0.15 rot = 52° at the encoder) is therefore ~27° of real arm travel, which matches the
   * CAD's 25° mate limit. So 1 code-radian = 24/46 arm-radians.
   */
  public static final double HOOD_ARM_RAD_PER_MECH_RAD = 24.0 / 46.0;

  /** Hard stops in code units. SHOOTER_HOOD_DEFAULT (0.0) is the resting position. */
  public static final double HOOD_HARD_STOP_DOWN_ROT = ShooterConstants.SHOOTER_HOOD_DEFAULT;
  public static final double HOOD_HARD_STOP_UP_ROT = ShooterConstants.SHOOTER_HOOD_MAX + 0.01;
  public static final double HOOD_START_ROT = ShooterConstants.SHOOTER_HOOD_DEFAULT;

  // ---------------------------------------------------------------------------------------------
  // Hopper extension — an outer box (side walls + front wall) that slides forward over the intake.
  // It starts the match slid in and deploys once, the first time the robot is enabled. No motor
  // in code, so this is purely a sim/visual model.
  // ---------------------------------------------------------------------------------------------

  /** Where the CAD draws the extension's front wall (robot frame x, inches). */
  private static final double HOPPER_FRONT_CAD_IN = 37.2;

  /**
   * Front wall when deployed. Measured off a side-view screenshot from the team: the wall sits
   * just past the deployed intake's front roller (25"), ~10" short of where the CAD draws it.
   */
  private static final double HOPPER_FRONT_OUT_IN = 27.2;

  /** ESTIMATE: front wall when slid in, at the frame front so it fits the frame perimeter. */
  private static final double HOPPER_FRONT_IN_IN = 14.0;

  /** Component offset from the CAD pose when fully out (negative = toward the robot's back). */
  public static final double HOPPER_OUT_OFFSET_M =
      Units.inchesToMeters(HOPPER_FRONT_OUT_IN - HOPPER_FRONT_CAD_IN);

  /** Slide travel between in and out. */
  public static final double HOPPER_TRAVEL_M =
      Units.inchesToMeters(HOPPER_FRONT_OUT_IN - HOPPER_FRONT_IN_IN);

  /** ESTIMATE: how long the extension takes to slide out. */
  public static final double HOPPER_DEPLOY_SECS = 0.5;

  /**
   * Collision box added to the MapleSim chassis while the hopper is out: from the bumper front
   * (MapleSim bumper is 30" long → x = 15") to the extension's front wall, 24" wide.
   */
  public static final double HOPPER_COLLISION_FRONT_X_M = Units.inchesToMeters(HOPPER_FRONT_OUT_IN);
  public static final double HOPPER_COLLISION_LENGTH_M =
      Units.inchesToMeters(HOPPER_FRONT_OUT_IN - 15.0);
  public static final double HOPPER_COLLISION_CENTER_X_M =
      Units.inchesToMeters((HOPPER_FRONT_OUT_IN + 15.0) / 2);
  public static final double HOPPER_COLLISION_WIDTH_M = Units.inchesToMeters(24.0);

  // ---------------------------------------------------------------------------------------------
  // Swerve wheels (AdvantageScope components 3-6, order FL, FR, BL, BR). Measured from the CAD by
  // tools/advantagescope/split-robot-glb.mjs, which prints these numbers — keep them in sync.
  // ---------------------------------------------------------------------------------------------

  /** Wheel centers (robot frame, meters). The wheels steer about a vertical axis through these. */
  public static final Translation3d[] WHEEL_CENTERS = {
    new Translation3d(0.2955, 0.2671, 0.0498),
    new Translation3d(0.2954, -0.2671, 0.0498),
    new Translation3d(-0.2895, 0.2671, 0.0498),
    new Translation3d(-0.2896, -0.2671, 0.0498),
  };

  /** Rolling direction each wheel is drawn at in the CAD (the modules are toed in an X). */
  public static final double[] WHEEL_CAD_STEER_RAD = {
    Math.toRadians(-45.4), Math.toRadians(44.6), Math.toRadians(44.6), Math.toRadians(-45.4),
  };

  // ---------------------------------------------------------------------------------------------
  // Game piece gating
  // ---------------------------------------------------------------------------------------------

  /**
   * The MapleSim intake zone only collects fuel when the wrist is within this many rotations of
   * fully down (≈11°). Prevents "intaking" while the arm is still swinging down.
   */
  public static final double INTAKE_DEPLOYED_TOLERANCE_ROT = 0.03;

  // ---------------------------------------------------------------------------------------------
  // Shots. Instead of guessing ball physics, the launch model is fit to the real robot's tuned
  // shot table (ShotConstants SHOOTER_SPEEDS / HOOD_ANGLES / SHOOTER_TOF): for each row, the launch
  // speed and angle that put the ball at the 1.83 m hub opening, d meters away, after exactly the
  // table's time of flight, using MapleSim's projectile gravity (11 m/s², no drag). Fit error is
  // ≤1.2° and every table row arrives within 0.11 m of the opening height. So with the flywheel and
  // hood where the table says, the sim scores, and it misses when they aren't settled yet.
  // ---------------------------------------------------------------------------------------------

  /** Ball exit speed (m/s) per flywheel rotor rps. Every table row gives 0.097-0.100. */
  public static final double SHOT_SPEED_MPS_PER_FLYWHEEL_RPS = 0.0986;

  /** Launch angle above horizontal = intercept + slope × hood position (code rotations). */
  public static final double SHOT_ANGLE_INTERCEPT_DEG = 58.81;
  public static final double SHOT_ANGLE_DEG_PER_HOOD_ROT = -50.37;

  /** Where the ball leaves the robot: behind center (matches ShotCalculator's launcher offset). */
  public static final double SHOT_EXIT_BEHIND_CENTER_M = 0.20;

  /** Exit height off the floor (same value the shot table generator used). */
  public static final double SHOT_EXIT_HEIGHT_M = 0.57;

  // ---------------------------------------------------------------------------------------------
  // Inside the robot: spindexer → feeder → flywheel (see FuelPathSim). Positions from CAD:
  // "Spinning floor" is a 21.2" disc centered at x = 3.6", top 5.7" up; the feeder is a 7.9"-wide
  // channel between the "Feeder Side Plate"s at the back (x -12.6" to -4.4"), rising to the
  // flywheel. A fuel ball drawn in CAD partway up the feeder marks the path's middle point.
  // ---------------------------------------------------------------------------------------------

  /** Center of the spindexer's spinning floor, at fuel-center height on it. */
  public static final Translation3d SPINDEXER_CENTER =
      new Translation3d(Units.inchesToMeters(3.6), 0.0, Units.inchesToMeters(5.7) + Units.inchesToMeters(5.91 / 2));

  /** Spinning floor radius (10.6"). */
  public static final double SPINDEXER_FLOOR_RADIUS_M = Units.inchesToMeters(10.6);

  /** Balls on the floor stay inside its rim: floor radius minus a fuel radius. */
  public static final double SPINDEXER_RING_RADIUS_M = Units.inchesToMeters(10.6 - 5.91 / 2);

  // Hopper box for the loose-ball physics (robot frame). Back wall at -5.4", extended front wall
  // at 27.2" (the hopper is out whenever it holds fuel), side walls at ±11.6".
  public static final double HOPPER_BACK_X_M = Units.inchesToMeters(-5.4);
  public static final double HOPPER_FRONT_X_M = Units.inchesToMeters(27.2);
  public static final double HOPPER_HALF_WIDTH_M = Units.inchesToMeters(11.6);

  /** The feeder/shooter tower fills the back of the hopper from this height up, to this x. */
  public static final double SHOOTER_BOTTOM_Z_M = 0.43;
  public static final double SHOOTER_FRONT_X_M = Units.inchesToMeters(1.8);

  /**
   * Outside the spinning floor (over the intake up front, and the hopper's corners) the hopper
   * floor is a funnel rising this much per meter away from the spindexer, so loose balls roll back
   * onto it. ESTIMATE: CAD has polycarb guides there; this just needs to be steep enough to roll.
   */
  public static final double HOPPER_RAMP_SLOPE = 0.6;

  /** Fed-in balls are let go here (robot frame; y follows where they were picked up). */
  public static final Translation3d HOPPER_DROP_POINT = new Translation3d(0.40, 0.0, 0.62);

  /**
   * ESTIMATE: how quickly the spinning floor's friction gets a resting ball moving with it
   * (1/s). Higher = balls lock to the floor faster.
   */
  public static final double SPINDEXER_GRIP_PER_SEC = 12.0;

  /**
   * ESTIMATE: outward push (m/s²) on balls riding the spinning floor, standing in for the center
   * cone and spin that send real balls to the rim, where the feeder entrance is.
   */
  public static final double SPINDEXER_FLING_MPS2 = 3.0;

  /** Velocity kept per physics substep (4 per loop); a little damping so piles settle. */
  public static final double HOPPER_DAMPING = 0.995;

  /** The feeder's funnel: balls whose centers get within this of its entrance (horizontally). */
  public static final double FEEDER_CAPTURE_RADIUS_M = 0.12;

  /** How fast the funnel draws a ball toward the feeder entrance (m/s). */
  public static final double FEEDER_FUNNEL_MPS = 0.8;

  /** Feeder path: entrance at the back of the ring → CAD ball partway up → flywheel contact. */
  public static final Translation3d[] FEEDER_PATH = {
    SPINDEXER_CENTER.plus(new Translation3d(-SPINDEXER_RING_RADIUS_M, 0.0, 0.0)),
    new Translation3d(Units.inchesToMeters(-6.2), 0.0, Units.inchesToMeters(17.9)),
    new Translation3d(-0.20, 0.0, 0.57), // = SHOT_EXIT_BEHIND_CENTER_M / SHOT_EXIT_HEIGHT_M
  };

  /**
   * ESTIMATE: fuel travel per feeder rotor revolution. Set so the feeder at its usual simulated
   * speed (~110 rps) moves fuel ~1.4 m/s; it takes the next ball once the last has climbed a
   * ball's width, which works out to the team's ~8 balls/s. Raise it to shoot faster.
   */
  public static final double FEEDER_BALL_M_PER_ROTOR_REV = 1.4 / 110.0;

  /**
   * ESTIMATE: spinning-floor revolutions per spindexer rotor revolution. ~1.25 rev/s at the usual
   * 80 rps, a little faster than the feeder can take balls so the feeder sets the fire rate.
   */
  public static final double SPINDEXER_REV_PER_ROTOR_REV = 1.8 / 80.0;

  /** How many fuel the real robot's hopper holds (team number). */
  public static final int HOPPER_CAPACITY = 65;

  /** Fuel is 5.91" across (game manual). */
  public static final double FUEL_RADIUS_M = Units.inchesToMeters(5.91 / 2);

  // ---------------------------------------------------------------------------------------------
  // Intake feeding (see IntakeSim). The deployed rollers sit ~22-25" ahead of robot center (CAD),
  // between the 21.5"-apart intake side plates.
  // ---------------------------------------------------------------------------------------------

  /** Fuel centers in this robot-relative box get picked up: from the bumper out past the roller. */
  public static final double INTAKE_ZONE_MIN_X_M = Units.inchesToMeters(15.0);
  public static final double INTAKE_ZONE_MAX_X_M = Units.inchesToMeters(28.5);
  public static final double INTAKE_ZONE_HALF_WIDTH_M = Units.inchesToMeters(10.0);

  /** ESTIMATE: how fast the intake can swallow fuel. Tune to the real robot. */
  public static final double INTAKE_BALLS_PER_SECOND = 10.0;

  /** Intake rollers must be spinning at least this fast (rotor rps) to pull fuel in. */
  public static final double INTAKE_ROLLER_INTAKING_RPS = 20.0;

  /** How long a ball takes to travel from the rollers into its hopper spot (drawing only). */
  public static final double FEED_ANIMATION_SECS = 0.3;

  /** The fed ball's arc bends toward this robot-relative point, just over the top roller. */
  public static final Translation3d FEED_ARC_OVER_ROLLER = new Translation3d(0.55, 0.0, 0.45);

  /**
   * Non-overlapping spots fuel starts from when the hopper is loaded all at once (tests, resets);
   * from there the balls fall and settle under the hopper physics. A loose grid over the hopper,
   * clear of the feeder and shooter at the back.
   */
  public static final Translation3d[] HOPPER_START_SPOTS = buildStartSpots();

  private static Translation3d[] buildStartSpots() {
    double[] xs = {-0.06, 0.09, 0.24, 0.39, 0.54};
    double[] ys = {-0.215, -0.072, 0.072, 0.215};
    double[] zs = {0.363, 0.513, 0.663, 0.813, 0.963};
    java.util.List<Translation3d> slots = new java.util.ArrayList<>();
    for (double z : zs) {
      for (double x : xs) {
        boolean inFeederOrShooter = x < 0.0 || (x < 0.12 && z > 0.43);
        if (inFeederOrShooter) continue;
        for (double y : ys) slots.add(new Translation3d(x, y, z));
      }
    }
    return slots.toArray(Translation3d[]::new);
  }
}
