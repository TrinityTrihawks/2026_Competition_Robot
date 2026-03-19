
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class KitbotSubsystem extends SubsystemBase {

  //set the SparkMaxes to the correct #, important.
  private DoubleSupplier getIntakespeed;
  private DoubleSupplier getIndexspeed;
  private DoubleSupplier getShootingspeed;

   private final SparkMax IntakeMotor = new SparkMax(21, MotorType.kBrushless);

   private final SparkMax IndexMotor = new SparkMax(23, MotorType.kBrushless);
   private final RelativeEncoder indexEncoder = IndexMotor.getEncoder();

   private final SparkMax ShootingMotor = new SparkMax(22,MotorType.kBrushless);

   // Stall detection state
   private boolean indexerActive = false;
   private boolean isReversing = false;
   private final Timer stallTimer = new Timer();
   private final Timer reverseTimer = new Timer();

   // Stall detection defaults
   private static final double DEFAULT_STALL_CURRENT_THRESHOLD = 20.0; // amps
   private static final double DEFAULT_STALL_VELOCITY_THRESHOLD = 50.0; // RPM
   private static final double DEFAULT_STALL_TIME_THRESHOLD = 0.3; // seconds before declaring stall
   private static final double DEFAULT_REVERSE_DURATION = 2.0; // seconds to reverse

   //Creates a new ExampleSubsystem.


   public KitbotSubsystem(DoubleSupplier getIndexSpeed, DoubleSupplier getIntakeSpeed, DoubleSupplier getShootingSpeed) {
      getIntakespeed = getIntakeSpeed;
      getIndexspeed = getIndexSpeed;
      getShootingspeed = getShootingSpeed;

      SmartDashboard.putNumber("Indexer Stall Current Threshold", DEFAULT_STALL_CURRENT_THRESHOLD);
      SmartDashboard.putNumber("Indexer Stall Velocity Threshold", DEFAULT_STALL_VELOCITY_THRESHOLD);
      SmartDashboard.putNumber("Indexer Stall Time Threshold", DEFAULT_STALL_TIME_THRESHOLD);
      SmartDashboard.putNumber("Indexer Reverse Duration", DEFAULT_REVERSE_DURATION);
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

   @Override
   public void periodic() {
     double current = IndexMotor.getOutputCurrent();
     double velocity = Math.abs(indexEncoder.getVelocity());
     double stallCurrentThreshold = SmartDashboard.getNumber("Indexer Stall Current Threshold", DEFAULT_STALL_CURRENT_THRESHOLD);
     double stallVelocityThreshold = SmartDashboard.getNumber("Indexer Stall Velocity Threshold", DEFAULT_STALL_VELOCITY_THRESHOLD);
     double stallTimeThreshold = SmartDashboard.getNumber("Indexer Stall Time Threshold", DEFAULT_STALL_TIME_THRESHOLD);
     double reverseDuration = SmartDashboard.getNumber("Indexer Reverse Duration", DEFAULT_REVERSE_DURATION);

     SmartDashboard.putNumber("Indexer Current", current);
     SmartDashboard.putNumber("Indexer Velocity", velocity);
     SmartDashboard.putBoolean("Indexer Stalled", isReversing);

     if (isReversing) {
       // Currently reversing - check if reverse duration has elapsed
       IndexMotor.set(-getIndexspeed.getAsDouble());
       if (reverseTimer.get() >= reverseDuration) {
         isReversing = false;
         stallTimer.reset();
         stallTimer.start();
       }
       return;
     }

     if (!indexerActive) {
       stallTimer.stop();
       stallTimer.reset();
       return;
     }

     // Check for stall: high current + low velocity
     boolean isStalled = current > stallCurrentThreshold && velocity < stallVelocityThreshold;

     if (isStalled) {
       if (stallTimer.get() == 0) {
         stallTimer.reset();
         stallTimer.start();
       }
       if (stallTimer.get() >= stallTimeThreshold) {
         // Stall confirmed - begin reversing
         isReversing = true;
         reverseTimer.reset();
         reverseTimer.start();
       }
     } else {
       stallTimer.reset();
       stallTimer.start();
     }
   }

   public void intakeToShoot() {
    IntakeMotor.set(getShootingspeed.getAsDouble()); // set as shooting speed so both dont fight
    ShootingMotor.set(-getShootingspeed.getAsDouble());
   }

   public void intakeToHopper(){
   indexerActive = true;
   ShootingMotor.set(-getIntakespeed.getAsDouble()); // runs to not drag intake motor
   IntakeMotor.set(getIntakespeed.getAsDouble());
   if (!isReversing) {
     IndexMotor.set(-getIndexspeed.getAsDouble());
   }
   }
   public void inverseEverything(){
   indexerActive = false; // manual reverse, skip stall detection
   ShootingMotor.set(getShootingspeed.getAsDouble());
   IndexMotor.set(getIndexspeed.getAsDouble());
   IntakeMotor.set(-getIntakespeed.getAsDouble());
   }
   public void revUpShooter() {
    ShootingMotor.set(-getShootingspeed.getAsDouble());
    IntakeMotor.set(getShootingspeed.getAsDouble());
   }
   public void engageIndexer() {
     indexerActive = true;
     if (!isReversing) {
       IndexMotor.set(getIndexspeed.getAsDouble());
     }
   }
   public void stopMotors() {
    indexerActive = false;
    isReversing = false;
    ShootingMotor.set(0);
    IndexMotor.set(0);
    IntakeMotor.set(0);
   }


}
