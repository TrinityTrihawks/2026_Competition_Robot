// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.HopperToShoot;
import frc.robot.commands.IntakeToHopper;
import frc.robot.commands.IntakeToShoot;
import frc.robot.commands.InverseEverything;
import frc.robot.commands.SmartShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitbotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
       private final KitbotSubsystem m_KitbotSubsystem = new KitbotSubsystem(
                       () -> SmartDashboard.getNumber("Speed in Voltage: Index Motor", 0.625),
                       () -> SmartDashboard.getNumber("Speed in Voltage: Intake Motor", 0.666),
                       () -> SmartDashboard.getNumber("Speed in Voltage: Shooting Motor", 0.45833));

        private DoubleSupplier speedSupplier = () -> SmartDashboard.getNumber("Swerve Drive Train Speed Percentage 0-1", 0.1);
        private DoubleSupplier angularSpeedSupplier = () -> SmartDashboard.getNumber("Swerve Drive Train Angular Rate 0-1", 0.1);
        
        

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second max angular velocity
        
        //private double ShootingSpeed;
        //private double IndexSpeed;
        //private double IntakeSpeed;
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * Constants.DEADBAND)
                        .withRotationalDeadband(MaxAngularRate * Constants.DEADBAND)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController subsController = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final VisionSubsystem m_vision = new VisionSubsystem(drivetrain);

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                NamedCommands.registerCommand("IntaketoShoot", new IntakeToShoot(m_KitbotSubsystem));
                NamedCommands.registerCommand("HoppertoShoot", new HopperToShoot(m_KitbotSubsystem));
                NamedCommands.registerCommand("IntaketoHopper", new IntakeToHopper(m_KitbotSubsystem));


                autoChooser = AutoBuilder.buildAutoChooser("");
                SmartDashboard.putData("Auto Mode", autoChooser);

                drivetrain.resetPose(new Pose2d(2, 1, new Rotation2d()));

                CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
                SmartDashboard.putNumber("Swerve Drive Train Speed Percentage 0-1", 0.3);
                SmartDashboard.putNumber("Swerve Drive Train Angular Rate 0-1", 0.4);
                SmartDashboard.putNumber("Speed in Voltage: Index Motor", 0.625);
                SmartDashboard.putNumber("Speed in Voltage: Intake Motor", 0.666);
                SmartDashboard.putNumber("Speed in Voltage: Shooting Motor",0.45833);

                CommandScheduler.getInstance().registerSubsystem(m_vision);

                configureBindings();

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(Math.pow(MathUtil.applyDeadband(-joystick.getLeftY(), 0.1), 3)
                                                                * MaxSpeed * speedSupplier.getAsDouble()) // Drive forward with negative
                                                                                          // y
                                                .withVelocityY(Math.pow(MathUtil.applyDeadband(-joystick.getLeftX(), 0.1), 3)
                                                                * MaxSpeed * speedSupplier.getAsDouble()) // Drive left with negative X
                                                                                          // (left)
                                                .withRotationalRate(Math.pow(MathUtil.applyDeadband(-joystick.getRightX(), 0.1), 3)
                                                                * MaxAngularRate * angularSpeedSupplier.getAsDouble()) // Drive
                                                                                                // counterclockwise with
                                                                                                // negative X
                                ));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                joystick.b().whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // Use POV controls for robot-centric movement
                joystick.povUp().whileTrue(
                                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
                joystick.povDown().whileTrue(
                                drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                joystick.rightTrigger().whileTrue(new AutoAlign(drivetrain, m_vision));

                drivetrain.registerTelemetry(logger::telemeterize);

               subsController.leftTrigger().whileTrue(new IntakeToShoot(m_KitbotSubsystem));
               subsController.rightTrigger().whileTrue(new HopperToShoot(m_KitbotSubsystem));
               subsController.povDown().whileTrue(new InverseEverything(m_KitbotSubsystem));
               subsController.x().whileTrue(new IntakeToHopper(m_KitbotSubsystem));
               subsController.rightBumper().whileTrue(new SmartShoot(drivetrain, m_vision, m_KitbotSubsystem, 3.5));
               subsController.leftBumper().whileTrue(new SmartShoot(drivetrain, m_vision, m_KitbotSubsystem, 6.5));
        }

        public Command getAutonomousCommand() {

                // return DynamicPathDemo.toFiringPosition();
                // PathPlannerPath dp = DynamicPathDemo.makePath();
                // return AutoBuilder.followPath(dp);
                return autoChooser.getSelected();
        }
}
