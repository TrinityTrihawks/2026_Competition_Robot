package frc.robot.commands;

import frc.robot.subsystems.KitbotSubsystem;

public class IntakeToShoot extends ShootingBase {

    public IntakeToShoot(KitbotSubsystem subsystem) {
        super(subsystem);
  }
  
  @Override
  public void execute() {
    m_subsystem.intakeToShoot();
  }
}