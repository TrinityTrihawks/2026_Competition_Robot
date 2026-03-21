package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.navigation.DynamicPathDemo;
import frc.robot.navigation.NavUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class AlignShooting extends SequentialCommandGroup{

    public AlignShooting(CommandSwerveDrivetrain drivetrain) {
        addCommands(
            Commands.defer( () -> 
        AutoBuilder.pathfindToPose(NavUtil.FindShootTarget(drivetrain.getState().Pose.getTranslation()),DynamicPathDemo.DEFAULT_CONSTRAINTS),
        Set.of(drivetrain)),
        new AlignToHub(drivetrain));
    }

    
}
