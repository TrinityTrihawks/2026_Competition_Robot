package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static final double DEADBAND = 0.05;
    public static final int DEFAULT_SPEED_SCALER = 1;
    public static final Translation2d BLUE_HUB = new Translation2d(4.63,4);
    public static final Translation2d RED_HUB = new Translation2d(11.91,4);
    public static final Pose2d BLUE_HUMAN = new Pose2d(0.05,0.45, Rotation2d.fromDegrees(180));
    public static final Pose2d RED_HUMAN = new Pose2d(16,7.5, Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_LEFT_CENTER = new Pose2d(7.8,4.4, Rotation2d.fromDegrees(-90));
    public static final Pose2d BLUE_RIGHT_CENTER = new Pose2d(7.8,3.6, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_LEFT_CENTER= new Pose2d(8.8,3.6, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_RIGHT_CENTER = new Pose2d(8.8,4.4, Rotation2d.fromDegrees(0));

    

    public static final Pose2d START_POSE = new Pose2d(3.5, 4, Rotation2d.fromDegrees(0));
    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight"; // navigate to Limeight Web Page to change which will be <limeight name>.local:5801
                                                                
        public static final double SINGLE_TAG_BASE_XY_STDDEV = 0.8;
        public static final double SINGLE_TAG_BASE_THETA_STDDEV = 1.2;

        public static final double MULTI_TAG_BASE_XY_STDDEV = 0.3;
        public static final double MULTI_TAG_BASE_THETA_STDDEV = 0.5;

        public static final double MAX_TAG_DISTANCE = 5.0;
        public static final double DISTANCE_SCALING_EXPONENT = 2.0;

    }
}
