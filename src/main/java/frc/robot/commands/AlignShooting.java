package frc.robot.commands;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.navigation.DynamicPathDemo;
import frc.robot.navigation.NavUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class AlignShooting extends SequentialCommandGroup {

    public enum AlignMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    private static final SendableChooser<AlignMode> alignModeChooser = new SendableChooser<>();

    static {
        alignModeChooser.setDefaultOption("Field-Centric", AlignMode.FIELD_CENTRIC);
        alignModeChooser.addOption("Robot-Centric", AlignMode.ROBOT_CENTRIC);
        SmartDashboard.putData(Constants.SmartDashboardConstants.KEY_ALIGN_MODE_CHOOSER, alignModeChooser);
    }

    public AlignShooting(CommandSwerveDrivetrain drivetrain) {
        addCommands(
                Commands.defer(() -> {
                    AlignMode mode = alignModeChooser.getSelected();
                    Command pathfind = AutoBuilder.pathfindToPose(
                            NavUtil.FindShootTarget(
                                    drivetrain.getState().Pose.getTranslation(),
                                    () -> SmartDashboard.getNumber(
                                            Constants.SmartDashboardConstants.KEY_TARGET_SHOOTING_DIST,
                                            Constants.SmartDashboardConstants.DEFAULT_TARGET_SHOOTING_DIST)),
                            DynamicPathDemo.DEFAULT_CONSTRAINTS);

                    if (mode == AlignMode.FIELD_CENTRIC) {
                        return pathfind.andThen(new FieldCentricAlign(drivetrain));
                    } else {
                        return pathfind
                                .andThen(new AlignToHub(drivetrain))
                                .andThen(new AlignDistance(drivetrain));
                    }
                }, Set.of(drivetrain)));
    }
}
