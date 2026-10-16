package frc.robot.utils;

import java.util.Optional;

import org.dyn4j.geometry.decompose.SweepLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team9410.configs.SweepConfig;
import frc.robot.Constants;
import frc.robot.constants.SweepConstants;
import frc.robot.utils.FieldUtils.GameZone;


public class SweepHelpers {
  
  public static SweepDirection convertButtonToSweep (ControllerButton key, boolean blueAlliance) {
    // hehe spagetti

    if (blueAlliance) {
      switch (key) {
        case X: return SweepDirection.TOP;
        case B: return SweepDirection.BOTTOM;
        case A: return SweepDirection.LEFT;
        default: return SweepDirection.RIGHT;
      }
    } else {
      switch (key) {
        case B: return SweepDirection.TOP;
        case X: return SweepDirection.BOTTOM;
        case Y: return SweepDirection.LEFT;
        default: return SweepDirection.RIGHT;
      }
    }
  }

  public static SequentialCommandGroup sweep (SweepDirection direction, GameZone zone, SweepConfig config) {
    if (zone != GameZone.NEUTRAL) {
      return new SequentialCommandGroup();
    }

    Pose2d dir = switch (direction) {
      case TOP:
        yield SweepConstants.TOP_TARGET;
      case BOTTOM:
        yield SweepConstants.BOTTOM_TARGET;
      case LEFT:
        yield SweepConstants.LEFT_TARGET;
      case RIGHT:
        yield SweepConstants.RIGHT_TARGET;
    };

    return new CommandBuilder(config.drive(), config.controller())
      .drive(dir)
      .build();
  }

  // based off when you look at the field from top-down with blue on left
  public enum SweepDirection {
    TOP,
    BOTTOM,
    LEFT,
    RIGHT
  }

  public enum ControllerButton {
    A,
    X,
    Y,
    B
  }

  public static Pose2d getTargetPose(SweepDirection direction) {
    switch (direction) {
      case TOP: return Constants.Sweep.TOP_TARGET;
      case BOTTOM: return Constants.Sweep.BOTTOM_TARGET;
      case LEFT: return Constants.Sweep.LEFT_TARGET;
      default: return Constants.Sweep.RIGHT_TARGET;
    }
  }

  public void update () {

  }
}
