package frc.robot.constants;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team9410.configs.LeadMotorConfig;
import frc.lib.team9410.configs.MotionMagicConfig;
import frc.lib.team9410.configs.MotorConfig;
import frc.lib.team9410.configs.VelocitySubsystemConfig;

public class FeederConstants {
    public static final int PRIMARY_CAN_ID = 40;
    public static final int SECONDARY_CAN_ID = 41;
    public static final int CANDI1_CAN_ID = 42;
    public static final int CANDI2_CAN_ID = 43;

    public static final double FEEDER_KP = 0.3;
    public static final double FEEDER_KI = 0;
    public static final double FEEDER_KD = 0;
    public static final double FEEDER_KG = 0;
    public static final double FEEDER_KS = 0.36803;
    public static final double FEEDER_KV = 0.098272;
    public static final double FEEDER_KA = 0.0032188;
    public static final double FEEDER_MM_ACCELERATION = 200;

    public static final List<MotorConfig> MOTOR_CONFIGS = List.of(
        MotorConfig.leader(PRIMARY_CAN_ID, NeutralModeValue.Coast),
        MotorConfig.follower(SECONDARY_CAN_ID, NeutralModeValue.Coast, true));

    public static final LeadMotorConfig FEEDER_LEAD_CONFIG = new LeadMotorConfig(
        FEEDER_KP, FEEDER_KI, FEEDER_KD, FEEDER_KG, Optional.of(FEEDER_KS), Optional.of(FEEDER_KV), Optional.of(FEEDER_KA), 1.0, 1.0);

    public static final MotionMagicConfig FEEDER_MOTION_MAGIC_CONFIG =
        MotionMagicConfig.forVelocity(FEEDER_MM_ACCELERATION);

    public static final VelocitySubsystemConfig FEEDER_CONFIG = new VelocitySubsystemConfig(
        MOTOR_CONFIGS, FEEDER_LEAD_CONFIG, FEEDER_MOTION_MAGIC_CONFIG, "Feeder");
        
  }

