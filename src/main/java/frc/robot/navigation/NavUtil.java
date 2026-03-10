package frc.robot.navigation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class NavUtil {
    private static double ShootingDist = 3.5; //meters



    public static Pose2d FindShootTarget(Translation2d RobotXY) {
        Translation2d OutputTranslation = Constants.BLUE_HUB.minus(RobotXY);
        double X = OutputTranslation.getX();
        double Y = OutputTranslation.getY();
        double theta = Math.atan2(Y, X);
        double x = ShootingDist * Math.cos(theta);
        double y = ShootingDist * Math.sin(theta);
        double targX = X - x;
        double targY = Y - y;
        Rotation2d TargHead= new Rotation2d(theta);
        Pose2d TargPose = new Pose2d(targX, targY, TargHead);
        return TargPose;
    } 
    
}
