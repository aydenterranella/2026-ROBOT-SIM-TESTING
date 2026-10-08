// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static frc.robot.sim.MechanismSimConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.StateMachine;
import java.util.List;

/**
 * Owns every mechanism sim on the robot and steps them once per loop.
 *
 * <p>Created in {@code Robot.simulationInit()} and updated from {@code Robot.simulationPeriodic()}.
 * Nothing in here runs on the real robot.
 *
 * <h2>What you can look at while the sim runs</h2>
 * <ul>
 *   <li><b>SmartDashboard "Sim/SideView"</b> — a {@link Mechanism2d} side view of the robot
 *       (front is on the right). Open it in Glass/AdvantageScope to watch the intake and hood move.
 *   <li><b>NT "Sim/Mechanisms/ComponentPoses"</b> — Pose3d[] for AdvantageScope 3D "components".
 *       Index 0 = intake arm, 1 = shooter arm. These rotate each part about its pivot relative to
 *       where it is drawn in CAD, so they line up with CAD models exported in the robot frame.
 *   <li><b>SmartDashboard "Sim/..."</b> numbers — simulated speeds, angles, and currents.
 * </ul>
 */
public class MechanismSimManager {
  private final RollerMechanismSim flywheel;
  private final RollerMechanismSim feeder;
  private final RollerMechanismSim spindexer;
  private final RollerMechanismSim intakeRoller;
  private final ArmMechanismSim intakeWrist;
  private final ArmMechanismSim hood;
  private final List<RollerMechanismSim> rollers;

  private final StructArrayPublisher<Pose3d> componentPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Sim/Mechanisms/ComponentPoses", Pose3d.struct)
          .publish();

  // Side-view drawing. Canvas is 1.6 m wide; robot center sits at x = 0.8 on the canvas.
  private static final double CANVAS_WIDTH_M = 1.6;
  private static final double CANVAS_HEIGHT_M = 1.2;
  private static final double CANVAS_ROBOT_CENTER_X = CANVAS_WIDTH_M / 2.0;
  private final Mechanism2d sideView = new Mechanism2d(CANVAS_WIDTH_M, CANVAS_HEIGHT_M);
  private final MechanismLigament2d intakeLigament;
  private final MechanismLigament2d hoodLigament;

  public MechanismSimManager(StateMachine sm) {
    // ----- Things that spin -----
    flywheel =
        new RollerMechanismSim(
            "Flywheel",
            sm.shooter.getVelocityMotor(),
            FLYWHEEL_GEARBOX,
            FLYWHEEL_J_KG_M2,
            FLYWHEEL_FRICTION_VOLTS);
    feeder =
        new RollerMechanismSim(
            "Feeder",
            sm.feeder.getVelocityMotor(),
            FEEDER_GEARBOX,
            FEEDER_J_KG_M2,
            FEEDER_FRICTION_VOLTS);
    spindexer =
        new RollerMechanismSim(
            "Spindexer",
            sm.spindexer.getVelocityMotor(),
            SPINDEXER_GEARBOX,
            SPINDEXER_J_KG_M2,
            SPINDEXER_FRICTION_VOLTS);
    intakeRoller =
        new RollerMechanismSim(
            "IntakeRoller",
            sm.intakeRoller.getVelocityMotor(),
            INTAKE_ROLLER_GEARBOX,
            INTAKE_ROLLER_J_KG_M2,
            INTAKE_ROLLER_FRICTION_VOLTS);
    rollers = List.of(flywheel, feeder, spindexer, intakeRoller);

    // ----- Things that pivot -----
    intakeWrist =
        new ArmMechanismSim(
            new ArmMechanismSim.Config(
                "IntakeWrist",
                IntakeConstants.WRIST_CONFIG,
                INTAKE_WRIST_GEARBOX,
                INTAKE_WRIST_J_KG_M2,
                INTAKE_ARM_LENGTH_M,
                INTAKE_REF_POSITION_ROT,
                INTAKE_REF_ANGLE_RAD,
                INTAKE_ARM_RAD_PER_MECH_RAD,
                INTAKE_HARD_STOP_DOWN_ROT,
                INTAKE_HARD_STOP_UP_ROT,
                INTAKE_START_ROT),
            sm.intakeWrist.getPositionMotor());
    hood =
        new ArmMechanismSim(
            new ArmMechanismSim.Config(
                "Hood",
                ShooterConstants.HOOD_CONFIG,
                HOOD_GEARBOX,
                HOOD_J_KG_M2,
                HOOD_ARM_LENGTH_M,
                HOOD_REF_POSITION_ROT,
                HOOD_REF_ANGLE_RAD,
                HOOD_ARM_RAD_PER_MECH_RAD,
                HOOD_HARD_STOP_DOWN_ROT,
                HOOD_HARD_STOP_UP_ROT,
                HOOD_START_ROT),
            sm.shooterHood.getPositionMotor());

    // ----- Side view -----
    // Chassis: a flat bar ~30" long at bumper height, just for reference.
    sideView
        .getRoot("Chassis", CANVAS_ROBOT_CENTER_X - 0.38, 0.10)
        .append(new MechanismLigament2d("Frame", 0.76, 0, 10, new Color8Bit(Color.kGray)));
    intakeLigament =
        sideView
            .getRoot(
                "IntakePivot",
                CANVAS_ROBOT_CENTER_X + INTAKE_PIVOT.getX(),
                INTAKE_PIVOT.getZ())
            .append(
                new MechanismLigament2d(
                    "IntakeArm", INTAKE_ARM_LENGTH_M, 0, 6, new Color8Bit(Color.kOrange)));
    hoodLigament =
        sideView
            .getRoot("HoodPivot", CANVAS_ROBOT_CENTER_X + HOOD_PIVOT.getX(), HOOD_PIVOT.getZ())
            .append(
                new MechanismLigament2d(
                    "ShooterArm", HOOD_ARM_LENGTH_M, 0, 6, new Color8Bit(Color.kLightBlue)));
    SmartDashboard.putData("Sim/SideView", sideView);
  }

  /** Call once per {@code simulationPeriodic()}. Steps physics, then publishes visuals. */
  public void update() {
    for (RollerMechanismSim roller : rollers) {
      roller.update(LOOP_PERIOD_SECS);
    }
    intakeWrist.update(LOOP_PERIOD_SECS);
    hood.update(LOOP_PERIOD_SECS);

    publishTelemetry();
  }

  /**
   * True once the intake arm has actually swung down. Used to gate the MapleSim intake zone, so
   * the robot can't pick up fuel while the arm is still stowed.
   */
  public boolean isIntakeDeployed() {
    return intakeWrist.getPositionRotations()
        <= IntakeConstants.INTAKE_MAX + INTAKE_DEPLOYED_TOLERANCE_ROT;
  }

  /** Simulated flywheel speed (rotor rps) — use this instead of the setpoint for realistic shots. */
  public double getFlywheelVelocityRps() {
    return flywheel.getVelocityRps();
  }

  private void publishTelemetry() {
    // Mechanism2d angles are degrees, measured from +X (robot front = right side of the canvas).
    intakeLigament.setAngle(Math.toDegrees(intakeWrist.getAngleRads()));
    hoodLigament.setAngle(Math.toDegrees(hood.getAngleRads()));

    componentPosePublisher.set(
        new Pose3d[] {
          rotateAboutPivot(INTAKE_PIVOT, intakeWrist.getAngleRads() - INTAKE_CAD_ANGLE_RAD),
          rotateAboutPivot(HOOD_PIVOT, hood.getAngleRads() - HOOD_CAD_ANGLE_RAD)
        });

    for (RollerMechanismSim roller : rollers) {
      SmartDashboard.putNumber("Sim/" + roller.getName() + "/VelocityRps", roller.getVelocityRps());
      SmartDashboard.putNumber("Sim/" + roller.getName() + "/CurrentAmps", roller.getCurrentDrawAmps());
    }
    for (ArmMechanismSim arm : List.of(intakeWrist, hood)) {
      SmartDashboard.putNumber("Sim/" + arm.getName() + "/AngleDeg", Math.toDegrees(arm.getAngleRads()));
      SmartDashboard.putNumber("Sim/" + arm.getName() + "/PositionRot", arm.getPositionRotations());
      SmartDashboard.putNumber("Sim/" + arm.getName() + "/CurrentAmps", arm.getCurrentDrawAmps());
    }
    SmartDashboard.putBoolean("Sim/IntakeDeployed", isIntakeDeployed());
  }

  /**
   * Pose that rotates a robot-frame model about {@code pivot} by {@code deltaRad} (+ = front end
   * up). Rotating "up" about the robot's +Y (left) axis is a NEGATIVE pitch in WPILib, hence the
   * minus sign.
   */
  private static Pose3d rotateAboutPivot(Translation3d pivot, double deltaRad) {
    return new Pose3d(pivot, new Rotation3d(0.0, -deltaRad, 0.0))
        .transformBy(new Transform3d(pivot.unaryMinus(), new Rotation3d()));
  }
}
