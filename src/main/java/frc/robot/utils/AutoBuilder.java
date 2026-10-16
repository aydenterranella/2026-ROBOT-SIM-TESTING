package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;

import com.google.common.collect.ImmutableMap;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;

public class AutoBuilder {

    private final Swerve drive;
    private final CommandXboxController controller;
    private final StateMachine stateMachine;

    public AutoBuilder (Swerve drive, CommandXboxController controller, StateMachine stateMachine) {
        this.drive = drive;
        this.controller = controller;
        this.stateMachine = stateMachine;
    }

    public SendableChooser<SequentialCommandGroup> build () {
        var map = ImmutableMap.<String, SequentialCommandGroup>builder()
            .put("RedHP",
                new SequentialCommandGroup(
                    new SwerveDriveCommand(this.drive, this.controller, true, AutoConstants.RED_LEFT_1, 1.0)
                )
                // new CommandBuilder(drive, controller)
                //     .drive(AutoConstants.RED_LEFT_1, 1.0)
                    // .drive(AutoConstants.RED_LEFT_2, 1.0)
                    // .drive(AutoConstants.RED_LEFT_3)
                    // .drive(AutoConstants.RED_LEFT_4)
                    // .command(new InstantCommand(
                    //     () -> {
                    //         stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
                    //         stateMachine.intakeRoller.setVelocity(125);
                    //     }
                    // ))
                    // .drive(AutoConstants.RED_LEFT_5)
                    // .command(new InstantCommand(
                    //     () -> {
                    //         stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
                    //         stateMachine.intakeRoller.brake();
                    //     }
                    // ))
                    // .drive(AutoConstants.RED_LEFT_6)
                    // .drive(AutoConstants.RED_LEFT_7)
                    // .drive(AutoConstants.RED_LEFT_8)
                    // .command(new InstantCommand(
                    //     () -> stateMachine.setWantedState(RobotState.SHOOTING)
                    // ))

            )
            // .put("RedDepot",
            //     new CommandBuilder(drive, controller)
            // )
            .build()
        ;

        var chooser = new SendableChooser<SequentialCommandGroup>();
        var keys = map.keySet().toArray();
        for (int i = 0; i < keys.length; i++) {
            // if (i == 0) {
            //     chooser.setDefaultOption((String) keys[0], map.get(keys[0]).build());
            //     continue;
            // }

            chooser.addOption((String) keys[i], map.get(keys[i]));
        }

        return chooser;
    }
}
