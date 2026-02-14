package frc.robot.commands;

import frc.robot.subsystems.KitbotSubsystem;

public class Shoot extends ShootingBase {

    public Shoot(KitbotSubsystem subsystem) {
        super(subsystem);
  }
  
  @Override
  public void execute() {
    m_subsystem.shoot();
  }
}