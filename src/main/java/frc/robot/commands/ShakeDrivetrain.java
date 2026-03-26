package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Rapidly shakes the drivetrain side-to-side (strafe) to help dislodge
 * balls stuck in the hopper during shooting.
 */
public class ShakeDrivetrain extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    private final Timer m_timer = new Timer();
    private boolean m_goingRight;

    public ShakeDrivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);

        SmartDashboard.putNumber("Shake Speed m/s", 0.3);
        SmartDashboard.putNumber("Shake Period Seconds", 0.1);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_goingRight = true;
    }

    @Override
    public void execute() {
        double period = SmartDashboard.getNumber("Shake Period Seconds", 0.1);
        double speed = SmartDashboard.getNumber("Shake Speed m/s", 0.3);

        if (m_timer.hasElapsed(period)) {
            m_goingRight = !m_goingRight;
            m_timer.restart();
        }

        double strafe = m_goingRight ? speed : -speed;

        m_drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(0.0)
                .withVelocityY(strafe)
                .withRotationalRate(0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(m_brake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
