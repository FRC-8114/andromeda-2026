package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.ImuMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.results.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    private final Limelight limelight;
    private final LimelightPoseEstimator megatag1PoseEstimator;
    private final LimelightPoseEstimator megatag2PoseEstimator;
    private final double stdDevFactor;

    public VisionIOLimelight(VisionConstants.CameraConfiguration cameraConfiguration) {
        this(
                cameraConfiguration.name(),
                new Pose3d(
                        cameraConfiguration.robotToCamera().getTranslation(),
                        cameraConfiguration.robotToCamera().getRotation()),
                cameraConfiguration.stdDeviation());
    }

    public VisionIOLimelight(String limelightName, Pose3d cameraOffset, double stdDevFactor) {
        limelight = new Limelight(limelightName);
        megatag1PoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG1);
        megatag2PoseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);
        this.stdDevFactor = stdDevFactor;

        var settings = limelight.getSettings()
                .withPipelineIndex(0)
                .withCameraOffset(cameraOffset);

        if (VisionConstants.USE_TAG_WHITELIST) {
            settings.withAprilTagIdFilter(Arrays.stream(VisionConstants.TAG_WHITELIST).boxed().toList());
        }

        settings.save();
        setIMUMode(VisionConstants.LIMELIGHT_IMU_MODE);
    }

    @Override
    public String getName() {
        return limelight.limelightName;
    }

    @Override
    public void setRobotState(RobotState robotState) {
        limelight.getSettings().withRobotOrientation(
                new Orientation3d(robotState.currentRotation(), robotState.currentAngularVelocity()));
    }

    @Override
    public void setIMUMode(ImuMode imuMode) {
        limelight.getSettings().withImuMode(imuMode).save();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        boolean hasHeartbeat = limelight.getNTTable().containsKey("hb");
        double heartbeatAgeSecs = hasHeartbeat
                ? Timer.getFPGATimestamp() - (limelight.getNTTable().getEntry("hb").getLastChange() / 1_000_000.0)
                : Double.POSITIVE_INFINITY;

        inputs.connected = hasHeartbeat && heartbeatAgeSecs <= VisionConstants.limelightHeartbeatTimeoutSecs;
        inputs.latestTargetObservation = new TargetObservation(
                Rotation2d.fromDegrees(limelight.getData().targetData.getHorizontalOffsetFromPrincipal()),
                Rotation2d.fromDegrees(limelight.getData().targetData.getVerticalOffsetFromPrincipal()));

        Optional<PoseEstimate> megatag1Estimate = megatag1PoseEstimator.getPoseEstimate();
        Optional<PoseEstimate> megatag2Estimate = megatag2PoseEstimator.getPoseEstimate();

        PoseObservation megatag1Observation = toPoseObservation(megatag1Estimate, stdDevFactor);
        PoseObservation megatag2Observation = toPoseObservation(megatag2Estimate, stdDevFactor);
        inputs.poseObservations = toObservationArray(megatag2Observation);
        inputs.seedObservations = toObservationArray(megatag1Observation);

        if (megatag2Estimate.isPresent() && megatag2Estimate.get().hasData) {
            inputs.tagIds = toTagIds(megatag2Estimate.get().rawFiducials);
        } else if (megatag1Estimate.isPresent() && megatag1Estimate.get().hasData) {
            inputs.tagIds = toTagIds(megatag1Estimate.get().rawFiducials);
        } else {
            inputs.tagIds = new int[0];
        }
    }

    private static PoseObservation toPoseObservation(Optional<PoseEstimate> poseEstimate, double stdDevFactor) {
        if (poseEstimate.isEmpty() || !poseEstimate.get().hasData) {
            return null;
        }

        PoseEstimate estimate = poseEstimate.get();
        return new PoseObservation(
                estimate.timestampSeconds,
                estimate.pose,
                estimate.getAvgTagAmbiguity(),
                estimate.tagCount,
                estimate.avgTagDist,
                stdDevFactor,
                estimate.isMegaTag2 ? PoseObservationType.MEGATAG_2 : PoseObservationType.MEGATAG_1);
    }

    private static PoseObservation[] toObservationArray(PoseObservation observation) {
        return observation != null ? new PoseObservation[] { observation } : new PoseObservation[0];
    }

    private static int[] toTagIds(RawFiducial[] rawFiducials) {
        int[] tagIds = new int[rawFiducials.length];
        for (int i = 0; i < rawFiducials.length; i++) {
            tagIds[i] = rawFiducials[i].id;
        }
        return tagIds;
    }
}
