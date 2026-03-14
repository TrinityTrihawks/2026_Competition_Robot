package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.KitbotSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperToShoot extends Command {

    private final KitbotSubsystem kitbot;
    private DoubleSupplier seconds;
    private final Timer timer = new Timer();
    private boolean indexEngaged = false;

    public HopperToShoot(KitbotSubsystem kitbot, DoubleSupplier seconds) {
        this.kitbot = kitbot;
        this.seconds = seconds;
        addRequirements(kitbot);
    }

    @Override
    public void initialize() {
        // runs ONCE when button is first pressed
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        kitbot.revUpShooter();
        // loops while button is held
        if (timer.get() >= seconds.getAsDouble()) {
            kitbot.engageIndexer();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // let whileTrue handle stopping
    }

    @Override
    public void end(boolean interrupted) {
        kitbot.stopMotors();
    }
}