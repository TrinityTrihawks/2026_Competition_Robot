package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignToHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController rotController = new PIDController(0.08, 0.0, 0.002);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public AlignToHub(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotController.enableContinuousInput(-Math.PI, Math.PI); // important for rotation
        rotController.setTolerance(Math.toRadians(2)); // 2 degrees tolerance
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d hub = Constants.BLUE_HUB;
        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();

        double targetAngle = Math.atan2(
            hub.getY() - robotPos.getY(),
            hub.getX() - robotPos.getX()
        );

        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();
        double rotOutput = rotController.calculate(currentAngle, targetAngle);

        drivetrain.setControl(drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotOutput));
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}