package frc.robot.constants;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team9410.configs.CancoderConfig;
import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;
import frc.lib.team9410.configs.PositionSubsystemConfig;
import frc.lib.team9410.configs.VelocitySubsystemConfig;

public class IntakeConstants {
  public static final int ROLLER_CAN_ID = 20;
  public static final int PRIMARY_PIVOT_CAN_ID = 21;
  public static final int SECONDARY_PIVOT_CAN_ID = 22;
  public static final int ENCODER_CAN_ID = 23;

  public static final double INTAKE_DOWN_SETPOINT = 0;
  public static final double INTAKE_UP_SETPOINT = 0;
  public static final double INTAKE_ROLLER_OUTPUT = 0.8;
  public static final double ROLLER_TARGET_RPS = 10;

  public static final double INTAKE_MIN = -0.09; // INTAKE UP
  public static final double INTAKE_MAX = -0.452; // INTAKE DOWN 0.44
  public static final double INTAKE_IDLE = -0.41;
  public static final double INTAKE_FEED = -0.25;
  public static final double INTAKE_DEFAULT = INTAKE_IDLE;

  // Roller velocity PID
  public static final double ROLLER_KP = 0.4;
  public static final double ROLLER_KI = 0;
  public static final double ROLLER_KD = 0;
  public static final double ROLLER_KG = 0;
  public static final double ROLLER_MM_ACCELERATION = 200;

  public static final List<MotorConfig> ROLLER_MOTOR_CONFIGS = List.of(
      MotorConfig.leader(ROLLER_CAN_ID, NeutralModeValue.Coast));

  public static final LeadMotorConfig ROLLER_LEAD_CONFIG = new LeadMotorConfig(
      ROLLER_KP, ROLLER_KI, ROLLER_KD, ROLLER_KG, Optional.empty(), Optional.empty(), Optional.empty(), 1.0, 1.0);

  public static final MotionMagicConfig ROLLER_MOTION_MAGIC_CONFIG =
      MotionMagicConfig.forVelocity(ROLLER_MM_ACCELERATION);

  // Wrist PID
  public static final double WRIST_KP = 30;
  public static final double WRIST_KI = 0;
  public static final double WRIST_KD = 0;
  public static final double WRIST_KG = 0;
  public static final double WRIST_SENSOR_TO_MECHANISM_RATIO = -1;
  public static final double WRIST_ROTOR_TO_SENSOR_RATIO = 44.444;
  public static final double WRIST_MM_CRUISE_VELOCITY = 1;
  public static final double WRIST_MM_ACCELERATION = 20;
  public static final double WRIST_MAGNET_OFFSET_ROTATIONS = -0.036;
  public static final double WRIST_DISCONTINUITY_POINT_ROTATIONS = 0.5;

  public static final List<MotorConfig> WRIST_MOTOR_CONFIGS = List.of(
      MotorConfig.leader(PRIMARY_PIVOT_CAN_ID, NeutralModeValue.Brake),
      MotorConfig.follower(SECONDARY_PIVOT_CAN_ID, NeutralModeValue.Coast, true));

  public static final LeadMotorConfig WRIST_LEAD_CONFIG = new LeadMotorConfig(
      WRIST_KP, WRIST_KI, WRIST_KD, WRIST_KG, Optional.empty(), Optional.empty(), Optional.empty(),
      WRIST_SENSOR_TO_MECHANISM_RATIO, WRIST_ROTOR_TO_SENSOR_RATIO);

  public static final CancoderConfig WRIST_CANCODER_CONFIG = new CancoderConfig(
      ENCODER_CAN_ID, WRIST_MAGNET_OFFSET_ROTATIONS, WRIST_DISCONTINUITY_POINT_ROTATIONS);

  public static final MotionMagicConfig WRIST_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
      WRIST_MM_CRUISE_VELOCITY, WRIST_MM_ACCELERATION);

  public static final PositionSubsystemConfig WRIST_CONFIG = new PositionSubsystemConfig(
      WRIST_MOTOR_CONFIGS, WRIST_LEAD_CONFIG, WRIST_CANCODER_CONFIG, WRIST_MOTION_MAGIC_CONFIG,
      "Intake Wrist", "degrees", Optional.of(INTAKE_DEFAULT));

  public static final VelocitySubsystemConfig ROLLER_CONFIG = new VelocitySubsystemConfig(
      ROLLER_MOTOR_CONFIGS, ROLLER_LEAD_CONFIG, ROLLER_MOTION_MAGIC_CONFIG, "Intake Roller");
}
