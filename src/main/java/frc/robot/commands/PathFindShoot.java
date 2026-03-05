// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.navigation.NavUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PathFindShoot extends Command {
  private double CurrX;
  private double CurrY;
  private Rotation2d CurrRot;
  private Translation2d CurrTrans;
  private Translation2d RobotPose;
  private final double MAX_DRIVE_SPEED = 1.5; // m/s
  private final double MAX_ROTATE_SPEED = 60; // degrees/s
  private final double Tolerance = 0.1; // distance tolerance meters
  private final double RotTolerance = 3; // rotational tolerance degrees
  private Pose2d CurrPose;
  private final SwerveDrivetrain m_drive;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final PIDController drivePID = new PIDController(0.8, 0, 0.05);
  private final PIDController turnPID = new PIDController(0.04, 0, 0.002);
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PathFindShoot(CommandSwerveDrivetrain drive) {
    m_drive = drive;
    drivePID.setTolerance(0.05);
    turnPID.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotPose = m_drive.getState().Pose.getTranslation();
    CurrPose = NavUtil.FindShootTarget(RobotPose);
    CurrTrans = CurrPose.getTranslation();
    CurrRot = CurrPose.getRotation();
    CurrX = CurrTrans.getX();
    CurrY = CurrTrans.getY();

    double DistError = Math.sqrt( CurrX * CurrX + CurrY * CurrY);

    double DriveOutput = drivePID.calculate(DistError, 0);
      DriveOutput = clamp(DriveOutput, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);
    double TurnOutput = drivePID.calculate(CurrRot.getDegrees(), 0);
      TurnOutput = clamp(TurnOutput, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED);

    m_drive.setControl(
      new SwerveRequest.RobotCentric()
        .withVelocityX(CurrX / DistError)
        .withVelocityY(CurrY / DistError)
        .withRotationalRate(TurnOutput)
    );



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setControl(brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (CurrX <= Tolerance && CurrY <= Tolerance && CurrRot.getDegrees() <= RotTolerance); {
      return true;
    }
    
  }
  private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}