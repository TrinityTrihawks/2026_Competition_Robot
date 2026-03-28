package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AlignDistance extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final PIDController distController = new PIDController(3.0, 0.0, 0.0);

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

    public AlignDistance(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        distController.setTolerance(0.05); // 5 cm tolerance
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Constants.BLUE_HUB : Constants.RED_HUB;

        double targetDist = SmartDashboard.getNumber(
                Constants.SmartDashboardConstants.KEY_TARGET_SHOOTING_DIST,
                Constants.SmartDashboardConstants.DEFAULT_TARGET_SHOOTING_DIST);

        Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
        double currentDist = robotPos.getDistance(hub);

        // Positive output = drive forward (toward hub)
        double output = distController.calculate(currentDist, targetDist);

        drivetrain.setControl(drive
                .withVelocityX(-output)
                .withVelocityY(0)
                .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return distController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
