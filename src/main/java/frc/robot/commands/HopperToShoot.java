package frc.robot.commands;

import frc.robot.subsystems.KitbotSubsystem;

public class HopperToShoot extends ShootingBase {

    public HopperToShoot(KitbotSubsystem subsystem) {
        super(subsystem);
  }
  
  @Override
  public void execute() {
    m_subsystem.hopperToShoot();
  }
}