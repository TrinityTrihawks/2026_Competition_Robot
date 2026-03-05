// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CommandSwerveDrivetrain m_drive;
  private final VisionSubsystem m_vision;
  double MAX_ROTATE_SPEED = 3.0; // radians
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private int tags;
    private double TX; 

    // Tunes how aggressively the robot rotates to face the tag
    private final PIDController m_anglePID = new PIDController(0.04, 0.0, 0.002);
    

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAlign(CommandSwerveDrivetrain drive, VisionSubsystem vision) {
    m_vision = vision;
    m_drive = drive;
    m_anglePID.setTolerance(1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tags = m_vision.getTagCount();
    TX = m_vision.getTagTx();

    if (tags < 1 ) {   // If no tags are seen stop moving
      m_drive.setControl(brake);
      return;
    }

    double rotateOutput = m_anglePID.calculate(TX, 0.0); //Finds next number for the PID controller
      rotateOutput = clamp(rotateOutput, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED); // Keeps the number within the min and max
      
      m_drive.setControl(
        new SwerveRequest.RobotCentric()
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(rotateOutput)
      );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_anglePID.atSetpoint(); // returns true when the output is within tolernace which is et andthe top of the file
  }
  private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}