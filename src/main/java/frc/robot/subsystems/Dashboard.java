// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.FieldUtils;

public class Dashboard extends SubsystemBase {
  private final NetworkTableInstance inst;
  private final NetworkTable table;
  private final NetworkTable drivingTable;
  private final NetworkTable testingTable;

  /** Creates a new Dashboard. */
  public Dashboard() {
    inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Scoring");
    drivingTable = inst.getTable("Driving PIDs");
    testingTable = inst.getTable("Robot Testing");
    
    // velos
    testingTable.getEntry("spindexerVelocity").setInteger(PowerRobotContainer.getData("SpindexerVelocity", 75));
    testingTable.getEntry("feederVelocity").setInteger(PowerRobotContainer.getData("FeederVelocity", 80));
    testingTable.getEntry("shooterVelocity").setInteger(PowerRobotContainer.getData("ShooterVelocity", 50));
    // hood target (rotations); user can edit on dashboard
    testingTable.getEntry("shooterHoodTarget").setDouble(Constants.Shooter.SHOOTER_HOOD_DEFAULT);
  }

  @Override
  public void periodic() {
    updateTestingDashboard();
  }

  public double getValue (String key) {
    return testingTable.getEntry(key).getDouble(0.0);
  }

  private void updateTestingDashboard() {
    testingTable.getEntry("robotState").setString(PowerRobotContainer.getData("robotState", "robotState is null"));
    // testingTable.getEntry("robotPose").setValue(PowerRobotContainer.getData("robotPose", "robotPose is null"));
    testingTable.getEntry("timeToShift").setDouble(PowerRobotContainer.getData("timeToShift", -1.0));

    testingTable.getEntry("matchTime").setDouble(DriverStation.getMatchTime());
    // testingTable.getEntry("matchLocation").setValue(DriverStation.getLocation().getAsInt());
    testingTable.getEntry("matchMessage").setString(DriverStation.getGameSpecificMessage());

    // pos
    testingTable.getEntry("shooterHoodEncoder").setDouble(PowerRobotContainer.getData("Shooter HoodPosition", -1.0));
    testingTable.getEntry("intakeWristEncoder").setDouble(PowerRobotContainer.getData("Intake WristPosition", -1.0));
    // hood target: read from dashboard and put in data map for left trigger / others
    double hoodTarget = testingTable.getEntry("shooterHoodTarget").getDouble(Constants.Shooter.SHOOTER_HOOD_DEFAULT);
    PowerRobotContainer.setData("Shooter HoodTarget", hoodTarget);

    // velo: display from data map and push same values into data map for RobotContainer/others
    // double shooterVel = testingTable.getEntry("shooterVelocity").getDouble(0.0);
    // double spindexerVel = testingTable.getEntry("spindexerVelocity").getDouble(0.0);
    // double feederVel = testingTable.getEntry("feederVelocity").getDouble(0.0);
    // double intakeRollerVel = PowerRobotContainer.getData("Intake RollerVelocity", 0.0);
    // double spindexerVel = PowerRobotContainer.getData("SpindexerVelocity", 0.0);
    // double feederVel = PowerRobotContainer.getData("FeederVelocity", 0.0);
    // testingTable.getEntry("shooterVelocity").setDouble(shooterVel);
    // testingTable.getEntry("intakeRollerVelocity").setDouble(intakeRollerVel);
    // testingTable.getEntry("spindexerVelocity").setDouble(spindexerVel);
    // testingTable.getEntry("feederVelocity").setDouble(feederVel);
    // PowerRobotContainer.setData("ShooterVelocity", shooterVel);
    // PowerRobotContainer.setData("Intake RollerVelocity", intakeRollerVel);
    // PowerRobotContainer.setData("SpindexerVelocity", spindexerVel);
    // PowerRobotContainer.setData("FeederVelocity", feederVel);

    // testingTable.getEntry("spindexerVelocity").setDouble(PowerRobotContainer.getData("SpindexerVelocity", 0.0));
    // testingTable.getEntry("feederVelocity").setDouble(PowerRobotContainer.getData("FeederVelocity", 0.0));
    // testingTable.getEntry("shooterVelocity").setDouble(PowerRobotContainer.getData("ShooterVelocity", 0.0));
  }
}