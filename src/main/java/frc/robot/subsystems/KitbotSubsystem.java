
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class KitbotSubsystem extends SubsystemBase {

  //set the SparkMaxes to the correct #, important.
  private DoubleSupplier getIntakespeed;
  private DoubleSupplier getIndexspeed;
  private DoubleSupplier getShootingspeed;

   private final SparkMax IntakeMotor = new SparkMax(21, MotorType.kBrushless);

   private final SparkMax IndexMotor = new SparkMax(23, MotorType.kBrushless);

   private final SparkMax ShootingMotor = new SparkMax(22,MotorType.kBrushless);

   //Creates a new ExampleSubsystem. 

   
   public KitbotSubsystem(DoubleSupplier getIndexSpeed, DoubleSupplier getIntakeSpeed, DoubleSupplier getShootingSpeed) {
      getIntakespeed = getIntakeSpeed;
      getIndexspeed = getIndexSpeed;
      getShootingspeed = getShootingSpeed;
  }
  
  
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
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

   public void intakeToShoot() {
    IntakeMotor.set(getShootingspeed.getAsDouble()); // set as shooting speed so both dont fight
    ShootingMotor.set(-getShootingspeed.getAsDouble());
   }

   public void intakeToHopper(){
   ShootingMotor.set(-getIntakespeed.getAsDouble()); // runs to not drag intake motor
   IntakeMotor.set(getIntakespeed.getAsDouble());
   IndexMotor.set(-getIndexspeed.getAsDouble());
   }
   public void inverseEverything(){
   ShootingMotor.set(getShootingspeed.getAsDouble());
   IndexMotor.set(getIndexspeed.getAsDouble());
   IntakeMotor.set(-getIntakespeed.getAsDouble());
   }
   public void revUpShooter() {
    ShootingMotor.set(-getShootingspeed.getAsDouble());
    IntakeMotor.set(getShootingspeed.getAsDouble());
   }
   public void engageIndexer() {
     IndexMotor.set(getIndexspeed.getAsDouble());
   }
   public void stopMotors() {
    ShootingMotor.set(0);
    IndexMotor.set(0);
    IntakeMotor.set(0);
   }


}
