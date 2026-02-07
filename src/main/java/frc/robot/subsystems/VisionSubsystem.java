package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this(drivetrain, VisionConstants.LIMELIGHT_NAME);
    }

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain, String limelightName) {
        m_drivetrain = drivetrain;
        m_limelightName = limelightName;
    }

    @Override
    public void periodic() {
        m_latestMeasurement = null;

        PoseEstimate estimate =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(m_limelightName);

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

        m_drivetrain.addVisionMeasurement(pose, estimate.timestampSeconds, stdDevs);

        m_latestMeasurement = new VisionMeasurement(pose, estimate.timestampSeconds, stdDevs);

        SmartDashboard.putNumber("Vision/TagCount", estimate.tagCount);
        SmartDashboard.putNumber("Vision/AvgTagDist", estimate.avgTagDist);
        SmartDashboard.putNumber("Vision/PoseX", pose.getX());
        SmartDashboard.putNumber("Vision/PoseY", pose.getY());
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
}
