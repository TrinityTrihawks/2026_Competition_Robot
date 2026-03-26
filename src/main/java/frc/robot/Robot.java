// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.telemetry.MatchTime;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  @Logged(name = "MatchTime")
  private final MatchTime matchTime = new MatchTime(2026);

  private static final NetworkTable alignTable = NetworkTableInstance.getDefault().getTable("Targeting");
  private static final DoublePublisher distancePub = alignTable.getDoubleTopic("DistanceToTarget").publish();
  private static final DoublePublisher distanceErrorPub = alignTable.getDoubleTopic("DistanceError").publish();
  private static final DoublePublisher anglePub = alignTable.getDoubleTopic("AngleToTargetDegCW").publish();

  public Robot() {
    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber(Constants.SmartDashboardConstants.KEY_BATTERY_VOLTAGE, RobotController.getBatteryVoltage());
    matchTime.update(MatchTime.kGameData2026.get());

    Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? Constants.BLUE_HUB : Constants.RED_HUB;
    Pose2d robotPose = m_robotContainer.drivetrain.getState().Pose;
    double distance = robotPose.getTranslation().getDistance(hub);
    double targetDist = SmartDashboard.getNumber(
            Constants.SmartDashboardConstants.KEY_TARGET_SHOOTING_DIST,
            Constants.SmartDashboardConstants.DEFAULT_TARGET_SHOOTING_DIST);
    distancePub.set(distance);
    distanceErrorPub.set(distance - targetDist);
    double targetAngleRad = Math.atan2(
            hub.getY() - robotPose.getY(),
            hub.getX() - robotPose.getX()
    );
    double currentAngleRad = robotPose.getRotation().getRadians();
    double errorRad = targetAngleRad - currentAngleRad;
    // Normalize to [-pi, pi]
    errorRad = Math.atan2(Math.sin(errorRad), Math.cos(errorRad));
    anglePub.set(Math.toDegrees(errorRad));
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
      SmartDashboard.putString(Constants.SmartDashboardConstants.KEY_AUTO_STARTING_POSE,
          String.format("(%.2f, %.2f) %.1f°",
              startPose.getX(), startPose.getY(),
              startPose.getRotation().getDegrees()));
      SmartDashboard.putNumberArray(Constants.SmartDashboardConstants.KEY_AUTO_STARTING_POSE_ARRAY,
          new double[] { startPose.getX(), startPose.getY(), startPose.getRotation().getDegrees() });
    } else {
      SmartDashboard.putString(Constants.SmartDashboardConstants.KEY_AUTO_STARTING_POSE, "N/A (not a PathPlanner auto)");
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
