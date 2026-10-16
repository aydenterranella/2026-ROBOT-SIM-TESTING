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
// import frc.lib.team9410.math.LinearInterpolator;

public class ShooterConstants {
    public static final int HOOD_CAN_ID = 50;
    public static final int HOOD_ENCODER_CAN_ID = 54;
    public static final int PRIMARY_FLYWHEELS_CAN_ID = 51;
    public static final int FOLLOWER_FLYWHEEL_1_CAN_ID = 52;
    public static final int FOLLOWER_FLYWHEEL_2_CAN_ID = 53;

    public static final double SHOOTER_HOOD_MIN = 0.005;
    public static final double SHOOTER_HOOD_MAX = 0.15;
    public static final double SHOOTER_HOOD_DEFAULT = 0.0; // default should be min

    // Flywheel velocity PID
    public static final double FLYWHEEL_KP = 0.0001;
    public static final double FLYWHEEL_KI = 0;
    public static final double FLYWHEEL_KD = 0;
    public static final double FLYWHEEL_KG = 0;
    public static final double FLYWHEEL_KS = 0.28159; //0.23295;
    public static final double FLYWHEEL_KV = 0.12365; //0.12161;
    public static final double FLYWHEEL_KA = 0.003086; //0.0030738;
    public static final double FLYWHEEL_MM_ACCELERATION = 200;

    public static final List<MotorConfig> FLYWHEEL_MOTOR_CONFIGS = List.of(
            MotorConfig.leader(PRIMARY_FLYWHEELS_CAN_ID, NeutralModeValue.Coast, true),
            MotorConfig.follower(FOLLOWER_FLYWHEEL_1_CAN_ID, NeutralModeValue.Coast, true),
            MotorConfig.follower(FOLLOWER_FLYWHEEL_2_CAN_ID, NeutralModeValue.Coast, true));

    public static final LeadMotorConfig FLYWHEEL_LEAD_CONFIG = new LeadMotorConfig(
            FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD, FLYWHEEL_KG, Optional.of(FLYWHEEL_KS), Optional.of(FLYWHEEL_KV), Optional.of(FLYWHEEL_KA), 1.0, 1.0);

    public static final MotionMagicConfig FLYWHEEL_MOTION_MAGIC_CONFIG = MotionMagicConfig
            .forVelocity(FLYWHEEL_MM_ACCELERATION);

    // Hood PID
    public static final double HOOD_KP = 40;
    public static final double HOOD_KI = 0;
    public static final double HOOD_KD = 0;
    public static final double HOOD_KG = 0;
    public static final double HOOD_SENSOR_TO_MECHANISM_RATIO = -1;
    public static final double HOOD_ROTOR_TO_SENSOR_RATIO = 60;
    public static final double HOOD_MM_CRUISE_VELOCITY = 15;
    public static final double HOOD_MM_ACCELERATION = 30;
    public static final double HOOD_MAGNET_OFFSET_ROTATIONS = 0.058;
    public static final double HOOD_DISCONTINUITY_POINT_ROTATIONS = 0.5;

    public static final List<MotorConfig> HOOD_MOTOR_CONFIGS = List.of(
            MotorConfig.leader(HOOD_CAN_ID, NeutralModeValue.Brake, false));

    public static final LeadMotorConfig HOOD_LEAD_CONFIG = new LeadMotorConfig(
            HOOD_KP, HOOD_KI, HOOD_KD, HOOD_KG, Optional.empty(), Optional.empty(), Optional.empty(),
            HOOD_SENSOR_TO_MECHANISM_RATIO, HOOD_ROTOR_TO_SENSOR_RATIO);

    public static final CancoderConfig HOOD_CANCODER_CONFIG = new CancoderConfig(
            HOOD_ENCODER_CAN_ID, HOOD_MAGNET_OFFSET_ROTATIONS, HOOD_DISCONTINUITY_POINT_ROTATIONS);

    public static final MotionMagicConfig HOOD_MOTION_MAGIC_CONFIG = new MotionMagicConfig(
            HOOD_MM_CRUISE_VELOCITY, HOOD_MM_ACCELERATION);

    public static final PositionSubsystemConfig HOOD_CONFIG = new PositionSubsystemConfig(
            HOOD_MOTOR_CONFIGS, HOOD_LEAD_CONFIG, HOOD_CANCODER_CONFIG, HOOD_MOTION_MAGIC_CONFIG,
            "Shooter Hood", "degrees", Optional.of(SHOOTER_HOOD_DEFAULT));

    public static final VelocitySubsystemConfig FLYWHEEL_CONFIG = new VelocitySubsystemConfig(
            FLYWHEEL_MOTOR_CONFIGS, FLYWHEEL_LEAD_CONFIG, FLYWHEEL_MOTION_MAGIC_CONFIG, "Shooter");

}
