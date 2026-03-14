package frc.robot.navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class NavUtil {
    private static final double ShootingDist = 2.9; //meters
    private static Translation2d TargetHub;

    private NavUtil() {}



    public static Pose2d FindShootTarget(Translation2d RobotXY) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            TargetHub = Constants.BLUE_HUB;
        }
        else{TargetHub = Constants.RED_HUB;
        }

        Translation2d OutputTranslation = TargetHub.minus(RobotXY);
        double X = OutputTranslation.getX();
        double Y = OutputTranslation.getY();
        double theta = Math.atan2(Y, X);

        double x = ShootingDist * Math.cos(theta);
        double y = ShootingDist * Math.sin(theta);

        double targX = TargetHub.getX() - x;
        double targY = TargetHub.getY() - y;

        Rotation2d TargHead = new Rotation2d(theta + Math.PI);
        Pose2d TargPose = new Pose2d(targX, targY, TargHead);
        return TargPose;
    } 
    public static Pose2d HumanStationPose() {
        Pose2d HumanStation;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            HumanStation = Constants.BLUE_HUMAN;
        }
        else {
            HumanStation = Constants.RED_HUMAN;
        }
        return HumanStation;
    }
    
}
