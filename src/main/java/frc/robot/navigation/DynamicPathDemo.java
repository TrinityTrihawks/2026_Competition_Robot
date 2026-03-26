package frc.robot.navigation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

public class DynamicPathDemo {
    public static final Pose2d blueBridgeLeft = new Pose2d(3.5, .6, Rotation2d.fromDegrees(0));
    public static final Pose2d blueBridgeRight = new Pose2d(5.7, .6, Rotation2d.fromDegrees(0));
    public static final Pose2d blueFiringPosition = new Pose2d(6.4, 4.0, Rotation2d.fromDegrees(180));
    public static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(5.0, 5.0, 2 * Math.PI, 4 * Math.PI);
    public static PathPlannerPath makePath() {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                blueBridgeLeft,
                blueBridgeRight,
                blueFiringPosition
        );

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        return path;
    }
    public static PathPlannerPath simple(Pose2d goal, double goalRotationDegrees) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(goal);

        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, Rotation2d.fromDegrees(goalRotationDegrees))
        );

        path.preventFlipping = true;
        return path;
    }
    
    public static Command toFiringPosition() {
        return AutoBuilder.pathfindToPose(blueFiringPosition, DEFAULT_CONSTRAINTS, 0);
    }
}
