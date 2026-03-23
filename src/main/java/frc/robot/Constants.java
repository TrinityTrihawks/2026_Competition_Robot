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
    public static final Pose2d RED_HUMAN = new Pose2d(16 , 7.35 , Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_LEFT_CENTER = new Pose2d(7.8,4.4, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_RIGHT_CENTER = new Pose2d(7.8,3.6, Rotation2d.fromDegrees(-90));
    

    

    public static final Pose2d START_POSE = new Pose2d(3.5, 4, Rotation2d.fromDegrees(0));
    public static final class SmartDashboardConstants {
        // Tunable keys and defaults
        public static final String KEY_INDEX_MOTOR_SPEED = "Speed% Index Motor";
        public static final double DEFAULT_INDEX_MOTOR_SPEED = 0.55;

        public static final String KEY_INTAKE_MOTOR_SPEED = "Speed% Intake Motor";
        public static final double DEFAULT_INTAKE_MOTOR_SPEED = 0.666;

        public static final String KEY_SHOOTING_MOTOR_SPEED = "Speed% Shooting Motor";
        public static final double DEFAULT_SHOOTING_MOTOR_SPEED = 0.87;

        public static final String KEY_SWERVE_SPEED = "Swerve Drive Train Speed Percentage 0-1";
        public static final double DEFAULT_SWERVE_SPEED = 0.45;

        public static final String KEY_SWERVE_ANGULAR_RATE = "Swerve Drive Train Angular Rate 0-1";
        public static final double DEFAULT_SWERVE_ANGULAR_RATE = 0.4;

        public static final String KEY_INDEXER_DELAY = "Indexer Delay: Seconds";
        public static final double DEFAULT_INDEXER_DELAY = 0.7;

        public static final String KEY_LIMELIGHT_CHOOSER = "Limelight On/Off";

        // Telemetry keys
        public static final String KEY_BATTERY_VOLTAGE = "Battery Voltage";
        public static final String KEY_AUTO_STARTING_POSE = "Auto/StartingPose";
        public static final String KEY_AUTO_STARTING_POSE_ARRAY = "Auto/StartingPoseArray";
        public static final String KEY_VISION_LIMELIGHT_CONNECTED = "Vision/LimelightConnected";
        public static final String KEY_VISION_TAG_COUNT = "Vision/TagCount";
        public static final String KEY_VISION_AVG_TAG_DIST = "Vision/AvgTagDist";
        public static final String KEY_ANGLE_ERROR_LIMELIGHT = "Angle Error for Limelight";
        public static final String KEY_TAGS = "tags";
    }

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
