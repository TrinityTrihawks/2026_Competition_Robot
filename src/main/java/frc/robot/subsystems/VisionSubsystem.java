package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {

    public record VisionMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs
    ) {}

    private final CommandSwerveDrivetrain m_drivetrain;
    private final String m_limelightName;
    private VisionMeasurement m_latestMeasurement = null;
    private StructPublisher<Pose2d> llPosePub;
    private double m_lastHeartbeat = 0;
    private int m_heartbeatStaleCount = 0;
    private final SendableChooser<Boolean> Limelight_On_Off = new SendableChooser<>();

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, VisionConstants.LIMELIGHT_NAME);
    }

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
         llPosePub = NetworkTableInstance.getDefault()
        .getStructTopic(limelightName + "-pose", Pose2d.struct).publish();
        m_drivetrain = drivetrain;
        m_limelightName = limelightName;

        Limelight_On_Off.setDefaultOption("On", true);
        Limelight_On_Off.addOption("Off", false);
    }

    @Override
    public void periodic() { // for periodic methods if it returns it cuts the loop back to the start
        m_latestMeasurement = null;

        Rotation2d rotation = m_drivetrain.getState().Pose.getRotation();

        double yaw = rotation.getDegrees();
        
        double YawRate = m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation(m_limelightName, yaw , YawRate, 0, 0, 0, 0);


        PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);

            llPosePub.set(estimate.pose); // publish the pose to network tables

        // Limelight connection status via heartbeat
        double heartbeat = LimelightHelpers.getHeartbeat(m_limelightName);
        if (heartbeat != m_lastHeartbeat) {
            m_lastHeartbeat = heartbeat;
            m_heartbeatStaleCount = 0;
        } else {
            m_heartbeatStaleCount++;
        }
        // If heartbeat hasn't changed in ~500ms (25 cycles at 50Hz), consider disconnected
        SmartDashboard.putBoolean(Constants.SmartDashboardConstants.KEY_VISION_LIMELIGHT_CONNECTED, m_heartbeatStaleCount < 25);

        SmartDashboard.putNumber(Constants.SmartDashboardConstants.KEY_VISION_TAG_COUNT, estimate.tagCount);
        SmartDashboard.putNumber(Constants.SmartDashboardConstants.KEY_VISION_AVG_TAG_DIST, estimate.avgTagDist);
        SmartDashboard.putNumber(Constants.SmartDashboardConstants.KEY_ANGLE_ERROR_LIMELIGHT, getTagTx());
        SmartDashboard.putNumber(Constants.SmartDashboardConstants.KEY_TAGS, getTagCount());

        if (estimate == null || estimate.tagCount == 0) {
            return;
        }


        if (estimate.avgTagDist > VisionConstants.MAX_TAG_DISTANCE) {
            return;
        }

        Pose2d pose = estimate.pose;
        if (pose.getX() < 0 || pose.getX() > 16.54
                || pose.getY() < 0 || pose.getY() > 8.21) {
            return;
        }

        Matrix<N3, N1> stdDevs = calculateStdDevs(estimate);

        if (Limelight_On_Off.getSelected() == true) {
        m_drivetrain.addVisionMeasurement(pose, estimate.timestampSeconds, stdDevs);
        }

        m_latestMeasurement = new VisionMeasurement(pose, estimate.timestampSeconds, stdDevs);
    }

    public VisionMeasurement getLatestMeasurement() {
        return m_latestMeasurement;
    }

    public boolean hasValidMeasurement() {
        return m_latestMeasurement != null;
    }

    private Matrix<N3, N1> calculateStdDevs(PoseEstimate estimate) {
        double baseXY;
        double baseTheta;

        if (estimate.tagCount >= 2) {
            baseXY = VisionConstants.MULTI_TAG_BASE_XY_STDDEV;
            baseTheta = VisionConstants.MULTI_TAG_BASE_THETA_STDDEV;
        } else {
            baseXY = VisionConstants.SINGLE_TAG_BASE_XY_STDDEV;
            baseTheta = VisionConstants.SINGLE_TAG_BASE_THETA_STDDEV;
        }

        double distFactor = Math.pow(
            Math.max(1.0, estimate.avgTagDist),
            VisionConstants.DISTANCE_SCALING_EXPONENT
        );

        return VecBuilder.fill(baseXY * distFactor, baseXY * distFactor, baseTheta * distFactor);
    }

    public double getAvgTagDist() {
    if (m_latestMeasurement == null) return -1.0;
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
    return (estimate != null && estimate.tagCount > 0) ? estimate.avgTagDist : -1.0;
    }

    public double getTagTx() {
    return LimelightHelpers.getTX(m_limelightName); // horizontal offset in degrees
    }
    public int getTagCount() {
        PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_limelightName);
    return estimate.tagCount ;
    }
}
