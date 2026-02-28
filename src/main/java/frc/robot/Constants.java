package frc.robot;

public class Constants {
    public static final double DEADBAND = 0.05;
    public static final int DEFAULT_SPEED_SCALER = 1;

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight-main";

        public static final double SINGLE_TAG_BASE_XY_STDDEV = 0.8;
        public static final double SINGLE_TAG_BASE_THETA_STDDEV = 1.2;

        public static final double MULTI_TAG_BASE_XY_STDDEV = 0.3;
        public static final double MULTI_TAG_BASE_THETA_STDDEV = 0.5;

        public static final double MAX_TAG_DISTANCE = 5.0;
        public static final double DISTANCE_SCALING_EXPONENT = 2.0;
    }
}
