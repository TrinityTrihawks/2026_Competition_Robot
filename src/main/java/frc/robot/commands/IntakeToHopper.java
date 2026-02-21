package frc.robot.commands;

import frc.robot.subsystems.KitbotSubsystem;

public class IntakeToHopper extends ShootingBase {

    public IntakeToHopper(KitbotSubsystem subsystem) {
        super(subsystem);
  }
  
  @Override
  public void execute() {
    m_subsystem.intakeToHopper();
  }
}