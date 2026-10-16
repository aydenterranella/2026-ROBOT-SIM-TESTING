package frc.lib.team9410.configs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Swerve;

public record SweepConfig(
    Swerve drive, 
    CommandXboxController controller,
    StateMachine stateMachine
) {}
