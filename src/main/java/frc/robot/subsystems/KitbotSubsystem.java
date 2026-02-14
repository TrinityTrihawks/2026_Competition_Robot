
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class KitbotSubsystem extends SubsystemBase {

  //set the SparkMaxes to the correct #, important.
  DoubleSupplier getIntakespeed;
  DoubleSupplier getIndexspeed;
  DoubleSupplier getShootingspeed;

   private final SparkMax IntakeMotor = new SparkMax(22, MotorType.kBrushless);

   private final SparkMax IndexMotor = new SparkMax(23, MotorType.kBrushless);

   private final SparkMax ShootingMotor = new SparkMax(21,MotorType.kBrushless);

   //Creates a new ExampleSubsystem. 

   
   public KitbotSubsystem(DoubleSupplier getIntakespeed, DoubleSupplier getIndexSpeed, DoubleSupplier getShootingSpeed) {
    this.getIntakespeed = getIntakespeed;
    this.getIndexspeed = getIndexSpeed;
    this.getShootingspeed = getShootingSpeed;
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

   public void intakeToHopper(){
   IntakeMotor.setVoltage(getIntakespeed.getAsDouble());
   IndexMotor.setVoltage(getIndexspeed.getAsDouble());
   }
   public void inverseEverything(){
   ShootingMotor.setVoltage(-getShootingspeed.getAsDouble());
   IndexMotor.setVoltage(-getIndexspeed.getAsDouble());
   IntakeMotor.setVoltage(-getIntakespeed.getAsDouble());

   }
   public void shoot() {
     ShootingMotor.setVoltage(getShootingspeed.getAsDouble());
     IndexMotor.setVoltage(-getIndexspeed.getAsDouble());
     IntakeMotor.setVoltage(getIntakespeed.getAsDouble());
   }
   public void stopMotors() {
    ShootingMotor.setVoltage(0);
    IndexMotor.setVoltage(0);
    IntakeMotor.setVoltage(0);
   }


}
