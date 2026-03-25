package frc.robot.navigation;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.DoubleSupplier;

public class NavUtil {
    private static Translation2d TargetHub = Constants.BLUE_HUB;

    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Targeting");
    private static final StructPublisher<Pose2d> targetPosePub =
            table.getStructTopic("ShootingTargetPose", Pose2d.struct).publish();
    private static Double SHOOTING_X_MAX = 3.25; // Don't shoot past this line

    // Calculates the terminal points of the shooting arc on the blue side. Flip for Red.
    public static List<Pose2d> findArcTerminalPoints(Double shootingDistance) {
        Double xh = Constants.BLUE_HUB.getX() - SHOOTING_X_MAX;
        Double yh = Math.sqrt(Math.pow(shootingDistance, 2) - Math.pow(xh, 2));
        Double theta = Math.atan2(yh, xh);
        Pose2d p1 = new Pose2d(SHOOTING_X_MAX, Constants.BLUE_HUB.getY() - yh, Rotation2d.fromRadians(theta));
        Pose2d p2 = new Pose2d(SHOOTING_X_MAX, Constants.BLUE_HUB.getY() + yh, Rotation2d.fromRadians(-theta));
        ArrayList<Pose2d> out = new ArrayList<>(2);
        out.add(p1);
        out.add(p2);
        return out;
    }

    public static Boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }
    public static Pose2d FindShootTarget(Translation2d RobotXY, DoubleSupplier SHOOTING_DOUBLE_SUPPLIER) {
        double shootingDistance = SHOOTING_DOUBLE_SUPPLIER.getAsDouble();
        Collection<Pose2d> terminalPoints = findArcTerminalPoints(shootingDistance);
        if (isBlue()) {
            TargetHub = Constants.BLUE_HUB;
        } else {
            TargetHub = Constants.RED_HUB;
            terminalPoints = terminalPoints.stream().map(FlippingUtil::flipFieldPose).toList();
        }
        Translation2d OutputTranslation = TargetHub.minus(RobotXY);
        double X = OutputTranslation.getX();
        double Y = OutputTranslation.getY();
        double theta = Math.atan2(Y, X);

        double x = SHOOTING_DOUBLE_SUPPLIER.getAsDouble() * Math.cos(theta);
        double y = SHOOTING_DOUBLE_SUPPLIER.getAsDouble() * Math.sin(theta);

        double targX = TargetHub.getX() - x;
        double targY = TargetHub.getY() - y;

        Rotation2d TargHead = new Rotation2d(theta);
        Pose2d TargPose = new Pose2d(targX, targY, TargHead);
        // Clip target pose to ends of the arc if it's out of bounds
        if (isBlue() && targX > SHOOTING_X_MAX || !isBlue() && targX < FlippingUtil.fieldSizeX - SHOOTING_X_MAX) {
            TargPose = TargPose.nearest(terminalPoints);
        }
        targetPosePub.set(TargPose);
        return TargPose;
    }

    public static Pose2d HumanStationPose() {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return Constants.BLUE_HUMAN;
        }
        return Constants.RED_HUMAN;
    }

}
