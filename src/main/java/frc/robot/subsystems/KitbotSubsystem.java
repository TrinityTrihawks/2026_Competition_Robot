

package frc.robot.subsystems;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class KitbotSubsystem extends SubsystemBase {
  //private final SparkMax IntakeMotor = new SparkMax(101, MotorType.kBrushless);

 // private final SparkMax IndexMotor = new SparkMax(102, MotorType.kBrushless);

  private final SparkMax ShootingMotor = new SparkMax(20, MotorType.kBrushless);
  // Creates a new ExampleSubsystem. */
  // public motorsubsystem() {}
   public KitbotSubsystem() {}
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

// public void IntakeMotor_run(double speed){
//   IntakeMotor.setVoltage(speed);
// }
// public void IndexMotor_run(double speed){
//   IndexMotor.setVoltage(speed);
// }
public void ShootingMotor_run(double speed){
 ShootingMotor.setVoltage(speed);
}

}




