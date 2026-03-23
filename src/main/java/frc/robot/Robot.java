// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Publish auto starting pose if the selected auto is a PathPlanner auto
    if (m_autonomousCommand instanceof PathPlannerAuto) {
      Pose2d startPose = ((PathPlannerAuto) m_autonomousCommand).getStartingPose();
      // PathPlanner autos are authored for blue; flip for red alliance
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        startPose = FlippingUtil.flipFieldPose(startPose);
      }
      SmartDashboard.putString("Auto/StartingPose",
          String.format("(%.2f, %.2f) %.1f°",
              startPose.getX(), startPose.getY(),
              startPose.getRotation().getDegrees()));
      SmartDashboard.putNumberArray("Auto/StartingPoseArray",
          new double[] { startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees() });
    } else {
      SmartDashboard.putString("Auto/StartingPose", "N/A (not a PathPlanner auto)");
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.initializeGyroPose();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
