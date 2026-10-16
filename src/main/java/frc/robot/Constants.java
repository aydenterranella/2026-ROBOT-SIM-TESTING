package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static final class OI extends frc.robot.constants.OIConstants {}
  public static final class CanBus extends frc.robot.constants.CanBusConstants {}
  public static final class Auto extends frc.robot.constants.AutoConstants {}
  public static final class Vision extends frc.robot.constants.VisionConstants {}
  public static final class Intake extends frc.robot.constants.IntakeConstants {}
  public static final class Spindexer extends frc.robot.constants.SpindexerConstants {}
  public static final class Feeder extends frc.robot.constants.FeederConstants {}
  public static final class Shooter extends frc.robot.constants.ShooterConstants {}
  public static final class Shot    extends frc.robot.constants.ShotConstants     {}
  public static final class Field   extends frc.robot.constants.FieldConstants    {}
  public static final class Sweep extends frc.robot.constants.SweepConstants {}
}
