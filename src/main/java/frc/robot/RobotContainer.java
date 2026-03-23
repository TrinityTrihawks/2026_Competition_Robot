// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;
import java.util.function.DoubleSupplier;

import choreo.util.ChoreoAllianceFlipUtil;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.HopperToShoot;
import frc.robot.commands.IntakeToHopper;
import frc.robot.commands.IntakeToShoot;
import frc.robot.commands.InverseEverything;
import frc.robot.generated.TunerConstants;
import frc.robot.navigation.DynamicPathDemo;
import frc.robot.navigation.NavUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitbotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static edu.wpi.first.units.Units.*;

public class RobotContainer {
    private final KitbotSubsystem m_KitbotSubsystem = new KitbotSubsystem(
            () -> SmartDashboard.getNumber("Speed%: Index Motor", 0.55),
            () -> SmartDashboard.getNumber("Speed%: Intake Motor", 0.666),
            () -> SmartDashboard.getNumber("Speed%: Shooting Motor", 0.87));

    private DoubleSupplier speedSupplier = () -> SmartDashboard.getNumber("Swerve Drive Train Speed Percentage 0-1", 0.45);
    private DoubleSupplier angularSpeedSupplier = () -> SmartDashboard.getNumber("Swerve Drive Train Angular Rate 0-1", 0.4);


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

    private final SendableChooser<Boolean> Left_or_RightChooser = new SendableChooser<>();

    private final SendableChooser<Command> autoChooser;

    // Add these three fields near your other drivetrain fields (e.g. below the `logger` line)
private final SlewRateLimiter xLimiter = new SlewRateLimiter(3); // units: fraction/sec
private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

    public RobotContainer() {
        NamedCommands.registerCommand("IntaketoShoot", new IntakeToShoot(m_KitbotSubsystem));
        NamedCommands.registerCommand("HoppertoShoot", new HopperToShoot(m_KitbotSubsystem, () -> SmartDashboard.getNumber("Indexer Delay: Seconds", 0.7)));
        NamedCommands.registerCommand("IntaketoHopper", new IntakeToHopper(m_KitbotSubsystem));
        NamedCommands.registerCommand("PathfindToShoot", Commands.defer( () -> 
        AutoBuilder.pathfindToPose(NavUtil.FindShootTarget(drivetrain.getState().Pose.getTranslation()),DynamicPathDemo.DEFAULT_CONSTRAINTS), 
        Set.of(drivetrain)));
        NamedCommands.registerCommand("PathfindToCenter2nd", PathfindToCenter2nd());


        autoChooser = AutoBuilder.buildAutoChooser("");
        autoChooser.addOption("PathfindingHumanStationtoShoot", AutoBuilderHumanStation());
        SmartDashboard.putData("Auto Mode", autoChooser);

        Left_or_RightChooser.setDefaultOption("Left", true);
        Left_or_RightChooser.addOption("Right", false);
        SmartDashboard.putData("Left or Right Auto Chooser", Left_or_RightChooser);

        initializeGyroPose();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        SmartDashboard.putNumber("Swerve Drive Train Speed Percentage 0-1", 0.45);
        SmartDashboard.putNumber("Swerve Drive Train Angular Rate 0-1", 0.40);
        SmartDashboard.putNumber("Speed%: Index Motor", 0.55);
        SmartDashboard.putNumber("Speed%: Intake Motor", 0.666);
        SmartDashboard.putNumber("Speed%: Shooting Motor", 0.87);
        SmartDashboard.putNumber("Indexer Delay: Seconds", 0.7);

        configureBindings();
        }
        public void initializeGyroPose() {
                drivetrain.resetPose(Constants.START_POSE);
        }
    


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                                .withVelocityX((MathUtil.applyDeadband(-joystick.getLeftY(), 0.1))
                                        * MaxSpeed * speedSupplier.getAsDouble()) // Drive forward with negative
                                // y
                                .withVelocityY((MathUtil.applyDeadband(-joystick.getLeftX(), 0.1))
                                        * MaxSpeed * speedSupplier.getAsDouble()) // Drive left with negative X
                                // (left)
                                .withRotationalRate((MathUtil.applyDeadband(-joystick.getRightX(), 0.1))
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
        joystick.leftBumper().onTrue(Commands.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.rightTrigger().whileTrue(Commands.defer( () -> 
        AutoBuilder.pathfindToPose(NavUtil.FindShootTarget(drivetrain.getState().Pose.getTranslation()),DynamicPathDemo.DEFAULT_CONSTRAINTS),
        Set.of(drivetrain)));

        drivetrain.registerTelemetry(logger::telemeterize);

        subsController.leftTrigger().whileTrue(new IntakeToShoot(m_KitbotSubsystem));
        subsController.rightTrigger().whileTrue(new HopperToShoot(m_KitbotSubsystem,() -> SmartDashboard.getNumber("Indexer Delay: Seconds", 0.7)));
        subsController.povDown().whileTrue(new InverseEverything(m_KitbotSubsystem));
        subsController.x().whileTrue(new IntakeToHopper(m_KitbotSubsystem));
    }

    public Command getAutonomousCommand() {
        // return AutoBuilder.pathfindToPose(new Pose2d(2 , 4, new Rotation2d(Degrees.zero())), DynamicPathDemo.DEFAULT_CONSTRAINTS);
        return autoChooser.getSelected();
    }

    public Command PathfindToCenter2nd() {
        return Commands.defer(() -> {
                Pose2d centerpose;
        
                if (Left_or_RightChooser.getSelected() == true) {
                        centerpose = Constants.BLUE_LEFT_CENTER;
                }
                else {
                        centerpose = Constants.BLUE_RIGHT_CENTER;
                }

        return AutoBuilder.pathfindToPose(centerpose,DynamicPathDemo.DEFAULT_CONSTRAINTS);},
        Set.of(drivetrain));
    }

    public Command AutoBuilderHumanStation() {
    return Commands.defer(() -> new SequentialCommandGroup(
        AutoBuilder.pathfindToPose(NavUtil.HumanStationPose(), DynamicPathDemo.DEFAULT_CONSTRAINTS),
        new WaitCommand(5),
        Commands.defer( () -> 
        AutoBuilder.pathfindToPose(NavUtil.FindShootTarget(drivetrain.getState().Pose.getTranslation()),DynamicPathDemo.DEFAULT_CONSTRAINTS),
        Set.of(drivetrain)),
        new HopperToShoot(m_KitbotSubsystem, () -> SmartDashboard.getNumber("Indexer Delay: Seconds", 0.7)).withTimeout(8)), 

        Set.of(drivetrain));
}
}
