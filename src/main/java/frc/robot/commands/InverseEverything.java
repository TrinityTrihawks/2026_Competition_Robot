package frc.robot.commands;

import frc.robot.subsystems.KitbotSubsystem;


public class InverseEverything extends ShootingBase {

    public InverseEverything(KitbotSubsystem subsystem) {
        super(subsystem);
}

 @Override
  public void execute() {
    m_subsystem.inverseEverything();
  }
}