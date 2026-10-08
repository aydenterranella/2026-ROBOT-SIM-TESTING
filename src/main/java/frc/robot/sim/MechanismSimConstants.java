// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
   * Reference pose for converting code units ↔ physical angle. In the CAD the intake is drawn
   * fully DEPLOYED with the arm ~2° below horizontal, and INTAKE_MAX is the "down" setpoint.
   * Physical angle convention: 0° = arm pointing straight forward, + = rotating up/back.
   */
  public static final double INTAKE_REF_POSITION_ROT = IntakeConstants.INTAKE_MAX;
  public static final double INTAKE_REF_ANGLE_RAD = Math.toRadians(-2.0);
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
  // Shooter "hood" — this is actually the WHOLE shooter arm (side plates, flywheel motors, hood
  // motor) tilting about the 4" FLYWHEEL's axle (Onshape mate "Revolute 1" in Shooter V2). The
  // flywheel sits on the pivot at the rear, so the hood stays the same distance from it.
  // Driven by 1x Kraken X44 on the arm; its 10t pinion climbs two FIXED 190t (10DP) gear sectors.
  // ---------------------------------------------------------------------------------------------
  public static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX44(1);

  /** Pivot = flywheel axle (11" hex shaft). CAD (robot frame): x=-0.273, z=0.517 m. */
  public static final Translation3d HOOD_PIVOT = new Translation3d(-0.272, 0.0, 0.517);

  /** Pivot → front end of the arm (where the motors are). CAD: ~11.3". */
  public static final double HOOD_ARM_LENGTH_M = 0.30;

  /**
   * ESTIMATE: shooter-arm MOI about the pivot. Most shooter parts have no material in CAD, so
   * this is a guess: ~3.5 kg (side plates 0.83 lb each + 3 X60s + X44) with CG ~0.2 m out.
   */
  public static final double HOOD_J_KG_M2 = 0.15;

  /**
   * The CAD draws the arm at the TOP of its travel, angled 15° above horizontal toward the front:
   * the hood pinion sits near the upper end of the 190t sector teeth, and the Onshape mate limits
   * are -25°..0° from that pose. So the arm's lowest point is 15° - 25° = -10°.
   *
   * <p>SHOOTER_HOOD_DEFAULT (0.0) is where the arm rests, so we anchor code position 0.0 to that
   * lowest point. Increasing code position raises the front of the arm. Physical angle
   * convention: 0° = arm pointing straight forward, + = front end rising.
   */
  public static final double HOOD_CAD_ANGLE_RAD = Math.toRadians(15.0);
  public static final double HOOD_TRAVEL_RAD = Math.toRadians(25.0);
  public static final double HOOD_REF_POSITION_ROT = ShooterConstants.SHOOTER_HOOD_DEFAULT;
  public static final double HOOD_REF_ANGLE_RAD = HOOD_CAD_ANGLE_RAD - HOOD_TRAVEL_RAD;

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
  // Game piece gating
  // ---------------------------------------------------------------------------------------------

  /**
   * The MapleSim intake zone only collects fuel when the wrist is within this many rotations of
   * fully down (≈11°). Prevents "intaking" while the arm is still swinging down.
   */
  public static final double INTAKE_DEPLOYED_TOLERANCE_ROT = 0.03;
}
