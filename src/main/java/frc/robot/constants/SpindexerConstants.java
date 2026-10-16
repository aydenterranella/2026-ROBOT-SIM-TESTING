package frc.robot.constants;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;
import frc.lib.team9410.configs.VelocitySubsystemConfig;

public class SpindexerConstants {
  public static final int CAN_ID = 30;
  public static final int LASER_1_CAN_ID = 31;
  public static final int LASER_2_CAN_ID = 32;

  public static final double SPINDEXER_KP = 0.1;
  public static final double SPINDEXER_KI = 0;
  public static final double SPINDEXER_KD = 0;
  public static final double SPINDEXER_KG = 0;
  public static final double SPINDEXER_MM_ACCELERATION = 200;
  public static final double SPINDEXER_KS = 0.30415;
  public static final double SPINDEXER_KV = 0.12429;
  public static final double SPINDEXER_KA = 0.0034288;

  public static final List<MotorConfig> SPINDEXER_MOTOR_CONFIGS = List.of(
      MotorConfig.leader(CAN_ID, NeutralModeValue.Brake));

  public static final LeadMotorConfig SPINDEXER_LEAD_CONFIG = new LeadMotorConfig(
      SPINDEXER_KP, SPINDEXER_KI, SPINDEXER_KD, SPINDEXER_KG, Optional.of(SPINDEXER_KS), Optional.of(SPINDEXER_KV), Optional.of(SPINDEXER_KA), 1.0, 1.0);

  public static final MotionMagicConfig SPINDEXER_MOTION_MAGIC_CONFIG =
      MotionMagicConfig.forVelocity(SPINDEXER_MM_ACCELERATION);

  public static final VelocitySubsystemConfig SPINDEXER_CONFIG = new VelocitySubsystemConfig(
      SPINDEXER_MOTOR_CONFIGS, SPINDEXER_LEAD_CONFIG, SPINDEXER_MOTION_MAGIC_CONFIG, "Spindexer");
}
