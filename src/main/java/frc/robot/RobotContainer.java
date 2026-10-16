// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team9410.PowerRobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.VelocitySysId;
import frc.robot.constants.AutoConstants;
import frc.robot.commands.StrafeCommand;
import frc.robot.commands.StrafeCommand.StrafeSide;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.utils.FieldUtils.GameZone;

public class RobotContainer implements PowerRobotContainer {

  // --- Other ---
  private final StateMachine stateMachine = new StateMachine();
  private AutoPath auto = AutoPath.BLUE_LEFT;
  /**
   * Game timer: counts up from 0 to 2 minutes 40 seconds (160 s). Start via
   * {@link #startGameTimer()}.
   */

  public static final double GAME_DURATION_SECONDS = 2 * 60 + 40; // 2:40

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  private final VelocitySysId shooterSysId = new VelocitySysId(stateMachine.shooter, "Shooter");
  private final VelocitySysId feederSysId = new VelocitySysId(stateMachine.feeder, "Feeder");
  private final VelocitySysId spindexerSysId = new VelocitySysId(stateMachine.spindexer, "Spindexer");

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putBoolean("Red Left Auto", true);
    SmartDashboard.putBoolean("Red Right Auto", false);
    SmartDashboard.putBoolean("Blue Left Auto", false);
    SmartDashboard.putBoolean("Blue Right Auto", false);

    // SysId: start log, run 4 tests per mechanism (quasistatic/dynamic, fwd/rev),
    // then stop log
    SmartDashboard.putData("SysId/Start Log", VelocitySysId.startLog());
    SmartDashboard.putData("SysId/Stop Log", VelocitySysId.stopLog());
    SmartDashboard.putData("SysId/Shooter Quasistatic Forward",
        shooterSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Quasistatic Reverse",
        shooterSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Shooter Dynamic Forward", shooterSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Dynamic Reverse", shooterSysId.dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Feeder Quasistatic Forward",
        feederSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Feeder Quasistatic Reverse",
        feederSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Feeder Dynamic Forward", feederSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Feeder Dynamic Reverse", feederSysId.dynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Spindexer Quasistatic Forward",
        spindexerSysId.quasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Spindexer Quasistatic Reverse",
        spindexerSysId.quasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Spindexer Dynamic Forward", spindexerSysId.dynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Spindexer Dynamic Reverse", spindexerSysId.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public void resetState() {
    stateMachine.setWantedState(RobotState.READY);
  }

  private void configureBindings() {
    // Intake in and out
    driverController.leftTrigger(0.5).and(() -> !driverController.rightTrigger(0.5).getAsBoolean())
        .onTrue(new InstantCommand(() -> stateMachine.setWantedState(RobotState.INTAKING)))
        .onFalse(new InstantCommand(() -> stateMachine.setWantedState(RobotState.READY)));
  
  //Shooting
    driverController.rightTrigger(0.5)
      .onTrue(new InstantCommand(()->{
        GameZone zone = stateMachine.getZoneFromPRC();
        if (zone == GameZone.NEUTRAL || zone == GameZone.INTERCHANGE) {
          stateMachine.setWantedState(RobotState.PASSING);
        } else {
          stateMachine.setWantedState(RobotState.SHOOTING);
        }
      }))
      .onFalse(new InstantCommand(() -> stateMachine.setWantedState(RobotState.READY)));

    //Outake
    driverController.leftBumper()
        .onTrue(new InstantCommand(() -> stateMachine.setWantedState(RobotState.OUTTAKING)))
        .onFalse(new InstantCommand(() -> stateMachine.setWantedState(RobotState.READY)));

    //Reset Gyro
      driverController.back().onTrue(new InstantCommand(
        () -> {
          stateMachine.resetGyro();
        }));

    // Strafe (hold button to hug nearest wall)
    driverController.y().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.FRONT));
    driverController.a().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.BACK));
    driverController.x().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.LEFT));
    driverController.b().whileTrue(new StrafeCommand(stateMachine.drivetrain, driverController, StrafeSide.RIGHT));

    //DriveTrain
      stateMachine.drivetrain.setDefaultCommand(new SwerveDriveCommand(stateMachine.drivetrain, driverController, false, stateMachine));

  }

  public AutoPath getAutoPathFromDash() {
    if (SmartDashboard.getBoolean("Red Left Auto", false) && auto != AutoPath.RED_LEFT) {
      return AutoPath.RED_LEFT;
    } else if (SmartDashboard.getBoolean("Red Right Auto", false) && auto != AutoPath.RED_RIGHT) {
      return AutoPath.RED_RIGHT;
    } else if (SmartDashboard.getBoolean("Blue Left Auto", false) && auto != AutoPath.BLUE_LEFT) {
      return AutoPath.BLUE_LEFT;
    } else if (SmartDashboard.getBoolean("Blue Right Auto", false) && auto != AutoPath.BLUE_RIGHT) {
      return AutoPath.BLUE_RIGHT;
    } else {
      return null;
    }
  }

  public void setAuto() {
    AutoPath newAuto = getAutoPathFromDash();
    if (newAuto == null) {
      return;
    }

    auto = newAuto;

    clearAutoSelections();
    switch (auto) {
      case RED_LEFT:
        SmartDashboard.putBoolean("Red Left Auto", true);
        break;
      case RED_RIGHT:
        SmartDashboard.putBoolean("Red Right Auto", true);
        break;
      case BLUE_LEFT:
        SmartDashboard.putBoolean("Blue Left Auto", true);
        break;
      case BLUE_RIGHT:
        SmartDashboard.putBoolean("Blue Right Auto", true);
        break;
    }
  }

  public Command getAutonomousCommand() {
    if (auto == null) {
      auto = AutoPath.BLUE_LEFT;
    }

    switch (auto) {
      case RED_LEFT:
        return getRedLeftAuto();
      case RED_RIGHT:
        return getRedRightAuto();
      case BLUE_LEFT:
        return getBlueLeftAuto();
      case BLUE_RIGHT:
        return getBlueRightAuto();
      default:
        return new InstantCommand(
            () -> System.out.println("Invalid auto"));
    }
  }

  public static enum AutoPath {
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT
  }

  // Builds the standard quadrant auto sequence with the given 7 poses. 
  private Command buildQuadrantAuto(
      Pose2d p1, Pose2d p2, Pose2d p3, Pose2d p4, Pose2d p5, Pose2d p6, Pose2d p7) {
    DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
    Translation2d hopperTarget =
        alliance == DriverStation.Alliance.Blue ? FieldConstants.HOPPER_BLUE : FieldConstants.HOPPER_RED;
    return new SequentialCommandGroup(
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p1, 6.0, 0.5),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p2, 6.0, 1.0, true),
        new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_MAX);
              stateMachine.intakeRoller.setVelocity(145);
            }),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p3, 12.0, 0.75),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p4, 6.0, 0.75),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p5, 6.0, 0.4),
        new InstantCommand(
            () -> {
              stateMachine.intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
              stateMachine.intakeRoller.brake();
            }),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p6, 3.0, 0.75),
        new SwerveDriveCommand(stateMachine.drivetrain, driverController, true, p7, 3.0, 1.0, true),
        new InstantCommand(() -> stateMachine.setWantedState(RobotState.SHOOTING)),
        new TurnToPointCommand(stateMachine.drivetrain, hopperTarget, 3),
        new WaitCommand(2.0));
  }

  public Command getRedLeftAuto() {
    return buildQuadrantAuto(
        AutoConstants.RED_LEFT_1, AutoConstants.RED_LEFT_2, AutoConstants.RED_LEFT_3,
        AutoConstants.RED_LEFT_4, AutoConstants.RED_LEFT_5, AutoConstants.RED_LEFT_6, AutoConstants.RED_LEFT_7);
  }

  public Command getRedRightAuto() {
    return buildQuadrantAuto(
        AutoConstants.RED_RIGHT_1, AutoConstants.RED_RIGHT_2, AutoConstants.RED_RIGHT_3,
        AutoConstants.RED_RIGHT_4, AutoConstants.RED_RIGHT_5, AutoConstants.RED_RIGHT_6, AutoConstants.RED_RIGHT_7);
  }

  public Command getBlueLeftAuto() {
    return buildQuadrantAuto(
        AutoConstants.BLUE_LEFT_1, AutoConstants.BLUE_LEFT_2, AutoConstants.BLUE_LEFT_3,
        AutoConstants.BLUE_LEFT_4, AutoConstants.BLUE_LEFT_5, AutoConstants.BLUE_LEFT_6, AutoConstants.BLUE_LEFT_7);
  }

  public Command getBlueRightAuto() {
    return buildQuadrantAuto(
        AutoConstants.BLUE_RIGHT_1, AutoConstants.BLUE_RIGHT_2, AutoConstants.BLUE_RIGHT_3,
        AutoConstants.BLUE_RIGHT_4, AutoConstants.BLUE_RIGHT_5, AutoConstants.BLUE_RIGHT_6, AutoConstants.BLUE_RIGHT_7);
  }

  public StateMachine getStateMachine() {
    return stateMachine;
  }

  public void clearAutoSelections() {
    SmartDashboard.putBoolean("Blue Right Auto", false);
    SmartDashboard.putBoolean("Blue Left Auto", false);
    SmartDashboard.putBoolean("Red Right Auto", false);
    SmartDashboard.putBoolean("Red Left Auto", false);
  }
}
