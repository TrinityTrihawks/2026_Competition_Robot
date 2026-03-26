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

/**
 * Aligns the robot to the hub by adjusting both translation and rotation
 * simultaneously using field-centric control. Drives directly toward the
 * target shooting position while rotating to face the hub.
 */
public class FieldCentricAlign extends Command {
    private final CommandSwerveDrivetrain drivetrain;

    private final PIDController rotController = new PIDController(10, 0.0, 0.002);
    private final PIDController xController = new PIDController(3.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(3.0, 0.0, 0.0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public FieldCentricAlign(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(Math.toRadians(2));
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
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

        // Calculate the target position at the desired distance from the hub
        Translation2d hubToRobot = robotPos.minus(hub);
        Translation2d targetPos = hub.plus(
                hubToRobot.div(hubToRobot.getNorm()).times(targetDist));

        // Calculate target angle to face the hub
        double targetAngle = Math.atan2(
                hub.getY() - robotPos.getY(),
                hub.getX() - robotPos.getX());

        double currentAngle = drivetrain.getState().Pose.getRotation().getRadians();

        double rotOutput = rotController.calculate(currentAngle, targetAngle);
        double xOutput = xController.calculate(robotPos.getX(), targetPos.getX());
        double yOutput = yController.calculate(robotPos.getY(), targetPos.getY());

        drivetrain.setControl(drive
                .withVelocityX(xOutput)
                .withVelocityY(yOutput)
                .withRotationalRate(rotOutput));
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint() && xController.atSetpoint() && yController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }
}
