// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team9410.configs;

import java.util.List;
import java.util.Optional;

/**
 * Single-parameter config for {@link frc.lib.team9410.subsystems.PositionSubsystem}.
 * Holds motor configs, lead/CANcoder/motion magic configs, and display name/units.
 */
public record PositionSubsystemConfig(
    List<MotorConfig> motorConfigs,
    LeadMotorConfig leadConfig,
    CancoderConfig cancoderConfig,
    MotionMagicConfig motionMagicConfig,
    String subsystemName,
    String units,
    Optional<Double> defaultPosition) {}
