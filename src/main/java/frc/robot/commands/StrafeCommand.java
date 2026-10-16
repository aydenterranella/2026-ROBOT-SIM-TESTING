// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

// import org.ejml.dense.row.SpecializedOps_DDRM;s

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.LocationConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.DriveUtil;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.GameZone;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StrafeCommand extends Command {
  public double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE = RotationsPerSecond.of(0.5)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit

  private final Swerve drivetrain;
  private final CommandXboxController controller;
  // Set in constructor or in initialize() when using the no-side constructor.
  private StrafeSide side = null;
  private final PIDController driveToPointController;
  private final double poseTolerance;
  private StrafeDirection direction = null;

  public StrafeCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      StrafeSide side) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.side = side;
    this.driveToPointController = new PIDController(6.0, 0, 0);
    this.poseTolerance = 6;
    addRequirements(drivetrain);
  }

  
  //Constructs with no side; the closest side to the robot is chosen when the
  //command starts.
   
  public StrafeCommand(Swerve drivetrain, CommandXboxController controller) {
    this(drivetrain, controller, null);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    StrafeSide thatSide = side;

    if (side == null) {
      thatSide = getClosestSide(drivetrain);
    }

    StrafeAxis axis = getStrafeAxis(thatSide);
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);

    if (zone != GameZone.INTERCHANGE) {
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain,
          MAX_ANGULAR_RATE, 0.0);
      final double coeff = OIConstants.MAX_SPEED_COEFFICIENT;

      System.out.println(axis.name());

      double speedX = axis == StrafeAxis.X ? speeds.vxMetersPerSecond * coeff : getXInput(axis, thatSide);
      double speedY = axis == StrafeAxis.Y ? speeds.vyMetersPerSecond * coeff : getYInput(axis, thatSide);

      drivetrain.drive(
          speedX,
          speedY,
          getRotation(axis != StrafeAxis.Y ? speeds.vxMetersPerSecond : speeds.vyMetersPerSecond, thatSide),
          Swerve.DriveMode.ROTATION_LOCK);
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double getRotation(double speed, StrafeSide side) {
    switch (side) {
      case FRONT:
        if (speed > 0.1) {
          direction = StrafeDirection.POSITIVE;
          return 45.0;
        } else if (speed < -0.1) {
          direction = StrafeDirection.NEGATIVE;
          return -45.0;
        } else if (direction == StrafeDirection.POSITIVE) {
          return 45.0;
        } else if (direction == StrafeDirection.NEGATIVE) {
          return -45.0;
        } else {
          return 0;
        }
      case LEFT:
        if (speed > 0.1) {
          direction = StrafeDirection.POSITIVE;
          return 45.0;
        } else if (speed < -0.1) {
          direction = StrafeDirection.NEGATIVE;
          return 135.0;
        } else if (direction == StrafeDirection.POSITIVE) {
          return 45.0;
        } else if (direction == StrafeDirection.NEGATIVE) {
          return 135.0;
        } else {
          return 90;
        }
      case RIGHT:
        if (speed > 0.1) {
          direction = StrafeDirection.POSITIVE;
          return -45.0;
        } else if (speed < -0.1) {
          direction = StrafeDirection.NEGATIVE;
          return -135.0;
        } else if (direction == StrafeDirection.POSITIVE) {
          return -45.0;
        } else if (direction == StrafeDirection.NEGATIVE) {
          return -135.0;
        } else {
          return -90;
        }
      case BACK:
        if (speed > 0.1) {
          direction = StrafeDirection.POSITIVE;
          return 135.0;
        } else if (speed < -0.1) {
          direction = StrafeDirection.NEGATIVE;
          return -135.0;
        } else if (direction == StrafeDirection.POSITIVE) {
          return 135.0;
        } else if (direction == StrafeDirection.NEGATIVE) {
          return -135.0;
        } else {
          return 180;
        }
      default:
        return 0.0;
    }
  }

  private double getCenterLine(GameZone zone) {
    switch (zone) {
      case BLUE_ALLIANCE:
        return LocationConstants.BLUE_ZONE_X_MID;
      case RED_ALLIANCE:
        return LocationConstants.RED_ZONE_X_MID;
      case NEUTRAL:
        return LocationConstants.NEUTRAL_X_MID;
      default:
        return -1;
    }
  }

  private double getYInput(StrafeAxis axis, StrafeSide side) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double strafeLine;

    if ((alliance == Alliance.Blue && side == StrafeSide.LEFT)
        ||
        (alliance == Alliance.Red && side == StrafeSide.RIGHT)) {
      strafeLine = LocationConstants.FAR_WALL_Y;
    } else {
      strafeLine = LocationConstants.NEAR_WALL_Y;
    }

    final Pose2d targetPose = new Pose2d(pose.getX(), strafeLine, pose.getRotation());
    final double directionMultiplier = alliance == Alliance.Blue ? -1.0 : 1.0;

    Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
        pose, targetPose, directionMultiplier, driveToPointController, poseTolerance);
        
    System.out.println("getx: " + velocity.getY());

    return velocity.getY();
  }

  private double getXInput(StrafeAxis axis, StrafeSide side) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double strafeLine;

    switch (zone) {
      case BLUE_ALLIANCE:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.BLUE_HUB_WALL_X;
        } else {
          strafeLine = LocationConstants.BLUE_ALLIANCE_WALL_X;
        }

        break;
      case RED_ALLIANCE:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.RED_ALLIANCE_WALL_X;
        } else {
          strafeLine = LocationConstants.RED_HUB_WALL_X;
        }

        break;

      case NEUTRAL:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.RED_CENTER_WALL_X;
        } else {
          strafeLine = LocationConstants.BLUE_CENTER_WALL_X;
        }

        break;

      default:
        strafeLine = 0.0;
        break;
    }

    System.out.println(zone);
    System.out.println(strafeLine);

    
    final Pose2d targetPose = new Pose2d(strafeLine, pose.getY(), pose.getRotation());
    final double directionMultiplier = alliance == Alliance.Blue ? -1.0 : 1.0;
    
    Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
      pose, targetPose, directionMultiplier, driveToPointController, poseTolerance);

    return velocity.getX();
  }

  
  //Returns the strafe side (wall) closest to the robot's current pose.
   //Uses alliance and zone to pick the correct wall X/Y constants.
  
  private static StrafeSide getClosestSide(Swerve drivetrain) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    double frontWallX;
    double backWallX;
    switch (zone) {
      case BLUE_ALLIANCE:
        frontWallX = LocationConstants.BLUE_HUB_WALL_X;
        backWallX = LocationConstants.BLUE_ALLIANCE_WALL_X;
        break;
      case RED_ALLIANCE:
        frontWallX = LocationConstants.RED_HUB_WALL_X;
        backWallX = LocationConstants.RED_ALLIANCE_WALL_X;
        break;
      case NEUTRAL:
        frontWallX = alliance == Alliance.Blue ? LocationConstants.RED_CENTER_WALL_X
            : LocationConstants.BLUE_CENTER_WALL_X;
        backWallX = alliance == Alliance.Blue ? LocationConstants.BLUE_CENTER_WALL_X
            : LocationConstants.RED_CENTER_WALL_X;
        break;
      default:
        return StrafeSide.FRONT;
    }

    // Far/near wall Y are fixed field positions; LEFT/RIGHT map by alliance (see
    // getXInput).
    double distFront = Math.abs(pose.getX() - frontWallX);
    double distBack = Math.abs(pose.getX() - backWallX);
    double distToFarWall = Math.abs(pose.getY() - LocationConstants.FAR_WALL_Y);
    double distToNearWall = Math.abs(pose.getY() - LocationConstants.NEAR_WALL_Y);

    double min = Math.min(Math.min(distFront, distBack), Math.min(distToFarWall, distToNearWall));
    if (min == distFront)
      return StrafeSide.FRONT;
    if (min == distBack)
      return StrafeSide.BACK;
    if (min == distToFarWall)
      return alliance == Alliance.Blue ? StrafeSide.LEFT : StrafeSide.RIGHT;
    return alliance == Alliance.Blue ? StrafeSide.RIGHT : StrafeSide.LEFT;
  }

  private StrafeAxis getStrafeAxis(StrafeSide side) {
    if (side == StrafeSide.LEFT || side == StrafeSide.RIGHT) {
      return StrafeAxis.X;
    } else {
      return StrafeAxis.Y;
    }
  }

  private static enum StrafeAxis {
    X,
    Y
  }

  private static enum StrafeDirection {
    POSITIVE,
    NEGATIVE
  }

  public static enum StrafeSide {
    LEFT,
    RIGHT,
    FRONT,
    BACK
  }
}