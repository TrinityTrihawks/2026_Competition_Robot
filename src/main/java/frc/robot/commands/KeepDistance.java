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
public class KeepDistance extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final CommandSwerveDrivetrain m_drivetrain;
    private VisionSubsystem m_vision;
    private final double m_targetDistanceMeters;

    // Tunes how aggressively the robot drives forward/back
    private final PIDController m_distancePID = new PIDController(0.8, 0.0, 0.05);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    // Tunes how aggressively the robot rotates to face the tag
    private final PIDController m_anglePID = new PIDController(0.04, 0.0, 0.002);

    private static final double MAX_DRIVE_SPEED = 1.5;   // m/s
    private static final double MAX_ROTATE_SPEED = 1.0;  // rad/s


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public KeepDistance(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, double targetDistanceMeters) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_targetDistanceMeters = targetDistanceMeters;

        m_distancePID.setTolerance(0.05);  // within 5cm is "good enough"
        m_anglePID.setTolerance(1.0);      // within 1 degree is "good enough"

        addRequirements(drivetrain);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curr_dist = m_vision.getAvgTagDist();
    double tx = m_vision.getTagTx();

    if (curr_dist < 0) {
      m_drivetrain.setControl(brake);
    }

    // Positive output = drive forward (toward tag), negative = back away
        double driveOutput = m_distancePID.calculate(curr_dist, m_targetDistanceMeters);
        driveOutput = clamp(driveOutput, -MAX_DRIVE_SPEED, MAX_DRIVE_SPEED);

        // tx > 0 means tag is to the right, so rotate right (negative for most conventions)
        double rotateOutput = m_anglePID.calculate(tx, 0.0);
        rotateOutput = clamp(rotateOutput, -MAX_ROTATE_SPEED, MAX_ROTATE_SPEED);

         // Robot-relative: X = forward, Y = strafe, Omega = rotation
        m_drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(driveOutput)
                .withVelocityY(0.0)
                .withRotationalRate(rotateOutput)
        );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(new SwerveRequest.Idle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}