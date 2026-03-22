package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.maxAmbiguity;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;
import frc.robot.subsystems.vision.VisionConstants.LimelightCameraConfiguration;
import frc.robot.subsystems.vision.VisionConstants.LimelightPoseMode;

public class VisionIOLimelight implements VisionIO {
    private record LimelightObservation(
            Pose3d pose,
            double timestampSeconds,
            int tagCount,
            double avgTagDist,
            double ambiguity,
            boolean isMegaTag2,
            boolean fiducialsValid,
            RawFiducialParseResult rawFiducials) {}

    private final String limelightName;
    private final LimelightCameraConfiguration config;
    private final DoubleArrayEntry megatag1Entry;
    private final DoubleArrayEntry megatag2Entry;
    private final NetworkTableEntry heartbeatEntry;
    private double lastHeartbeat = Double.NaN;
    private double lastHeartbeatUpdateTimeSeconds = Double.NEGATIVE_INFINITY;
    private double lastMeasurementTimestampSeconds = Double.NEGATIVE_INFINITY;
    private double lastSeedTimestampSeconds = Double.NEGATIVE_INFINITY;

    public VisionIOLimelight(LimelightCameraConfiguration camera) {
        config = camera;
        limelightName = camera.name();
        megatag1Entry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "botpose_wpiblue");
        megatag2Entry = LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, "botpose_orb_wpiblue");
        heartbeatEntry = LimelightHelpers.getLimelightNTTableEntry(limelightName, "hb");

        var cameraTranslation = camera.robotToCamera().getTranslation();
        var cameraRotation = camera.robotToCamera().getRotation();

        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                cameraTranslation.getX(),
                cameraTranslation.getY(),
                cameraTranslation.getZ(),
                Units.radiansToDegrees(cameraRotation.getX()),
                Units.radiansToDegrees(cameraRotation.getY()),
                Units.radiansToDegrees(cameraRotation.getZ()));
        LimelightHelpers.SetIMUMode(limelightName, VisionConstants.LIMELIGHT_IMU_MODE);
    }

    @Override
    public CameraConfiguration getConfiguration() {
        return config;
    }

    @Override
    public Optional<PoseEstimation> sampleSeedPose() {
        double nowSeconds = Timer.getFPGATimestamp();
        if (!updateConnectionState(nowSeconds)) {
            return Optional.empty();
        }

        return getObservation(LimelightPoseMode.MEGATAG1)
                .filter(observation -> isFresh(observation, nowSeconds))
                .filter(observation -> isNewTimestamp(observation.timestampSeconds(), lastSeedTimestampSeconds))
                .filter(observation -> !shouldRejectEstimate(observation, false))
                .map(observation -> {
                    lastSeedTimestampSeconds = observation.timestampSeconds();
                    return VisionIO.PoseEstimation.seedFromStatistics(
                            observation.pose(),
                            Seconds.of(observation.timestampSeconds()),
                            observation.ambiguity());
                });
    }

    private Optional<LimelightObservation> getObservation(LimelightPoseMode mode) {
        TimestampedDoubleArray atomic = getObservationEntry(mode).getAtomic();
        double[] poseArray = atomic.value;
        boolean isMegaTag2 = mode == LimelightPoseMode.MEGATAG2;

        if (poseArray.length < 11) {
            return Optional.empty();
        }

        double latencyMilliseconds = getEntry(poseArray, 6);
        int tagCount = (int) getEntry(poseArray, 7);
        if (tagCount <= 0) {
            return Optional.empty();
        }

        RawFiducialParseResult rawFiducials = parseRawFiducials(poseArray, tagCount);
        double ambiguity = getAverageTagAmbiguity(rawFiducials);
        double timestampSeconds = (atomic.timestamp / 1_000_000.0) - (latencyMilliseconds / 1_000.0);
        if (timestampSeconds <= 0.0) {
            return Optional.empty();
        }

        return Optional.of(new LimelightObservation(
                LimelightHelpers.toPose3D(poseArray),
                timestampSeconds,
                tagCount,
                getEntry(poseArray, 9),
                ambiguity,
                isMegaTag2,
                rawFiducials.valid(),
                rawFiducials));
    }

    private DoubleArrayEntry getObservationEntry(LimelightPoseMode mode) {
        return switch (mode) {
            case MEGATAG1 -> megatag1Entry;
            case MEGATAG2 -> megatag2Entry;
        };
    }

    private record RawFiducialParseResult(RawFiducial[] fiducials, boolean valid) {}

    private record FiducialLogData(
            String sampleId,
            String cameraId,
            int tagId,
            double tx,
            double ty,
            double tz,
            double qx,
            double qy,
            double qz,
            double qw) {}

    private RawFiducialParseResult parseRawFiducials(double[] poseArray, int tagCount) {
        int valuesPerFiducial = 7;
        int expectedLength = 11 + (valuesPerFiducial * tagCount);
        if (poseArray.length != expectedLength) {
            return new RawFiducialParseResult(new RawFiducial[0], false);
        }

        RawFiducial[] rawFiducials = new RawFiducial[tagCount];
        for (int i = 0; i < tagCount; i++) {
            int baseIndex = 11 + (i * valuesPerFiducial);
            rawFiducials[i] = new RawFiducial(
                    (int) poseArray[baseIndex],
                    poseArray[baseIndex + 1],
                    poseArray[baseIndex + 2],
                    poseArray[baseIndex + 3],
                    poseArray[baseIndex + 4],
                    poseArray[baseIndex + 5],
                    poseArray[baseIndex + 6]);
        }
        return new RawFiducialParseResult(rawFiducials, true);
    }

    private void clearFiducialInputs(VisionIOInputs inputs) {
        inputs.fiducialSampleIds = new String[] {};
        inputs.fiducialCameraIds = new String[] {};
        inputs.fiducialTagIds = new int[] {};
        inputs.fiducialTx = new double[] {};
        inputs.fiducialTy = new double[] {};
        inputs.fiducialTz = new double[] {};
        inputs.fiducialQx = new double[] {};
        inputs.fiducialQy = new double[] {};
        inputs.fiducialQz = new double[] {};
        inputs.fiducialQw = new double[] {};
    }

    private void setCameraExtrinsicInputs(VisionIOInputs inputs) {
        var robotToCamera = config.robotToCamera();
        var quaternion = robotToCamera.getRotation().getQuaternion();
        inputs.cameraId = config.name();
        inputs.extrinsicTx = robotToCamera.getX();
        inputs.extrinsicTy = robotToCamera.getY();
        inputs.extrinsicTz = robotToCamera.getZ();
        inputs.extrinsicQx = quaternion.getX();
        inputs.extrinsicQy = quaternion.getY();
        inputs.extrinsicQz = quaternion.getZ();
        inputs.extrinsicQw = quaternion.getW();
    }

    private String getSampleId(double timestampSeconds) {
        return Long.toString(Math.round(timestampSeconds * 1_000_000.0));
    }

    private String getSampleId(LimelightHelpers.LimelightResults results, double fallbackTimestampSeconds) {
        if (results != null && results.timestamp_us > 0) {
            return Long.toString(results.timestamp_us);
        }
        return getSampleId(fallbackTimestampSeconds);
    }

    private List<FiducialLogData> getFiducialLogData(
            LimelightHelpers.LimelightResults results,
            double fallbackTimestampSeconds) {
        if (results == null || results.targets_Fiducials == null || results.targets_Fiducials.length == 0) {
            return List.of();
        }

        String sampleId = getSampleId(results, fallbackTimestampSeconds);
        List<FiducialLogData> fiducialLogData = new ArrayList<>();

        for (LimelightHelpers.LimelightTarget_Fiducial fiducial : results.targets_Fiducials) {
            Pose3d cameraToTag = fiducial.getTargetPose_CameraSpace();
            var quaternion = cameraToTag.getRotation().getQuaternion();
            fiducialLogData.add(new FiducialLogData(
                    sampleId,
                    config.name(),
                    (int) fiducial.fiducialID,
                    cameraToTag.getX(),
                    cameraToTag.getY(),
                    cameraToTag.getZ(),
                    quaternion.getX(),
                    quaternion.getY(),
                    quaternion.getZ(),
                    quaternion.getW()));
        }

        return fiducialLogData;
    }

    private void setFiducialInputs(VisionIOInputs inputs, List<FiducialLogData> fiducialLogData) {
        int count = fiducialLogData.size();
        inputs.fiducialSampleIds = new String[count];
        inputs.fiducialCameraIds = new String[count];
        inputs.fiducialTagIds = new int[count];
        inputs.fiducialTx = new double[count];
        inputs.fiducialTy = new double[count];
        inputs.fiducialTz = new double[count];
        inputs.fiducialQx = new double[count];
        inputs.fiducialQy = new double[count];
        inputs.fiducialQz = new double[count];
        inputs.fiducialQw = new double[count];

        for (int i = 0; i < count; i++) {
            FiducialLogData fiducial = fiducialLogData.get(i);
            inputs.fiducialSampleIds[i] = fiducial.sampleId();
            inputs.fiducialCameraIds[i] = fiducial.cameraId();
            inputs.fiducialTagIds[i] = fiducial.tagId();
            inputs.fiducialTx[i] = fiducial.tx();
            inputs.fiducialTy[i] = fiducial.ty();
            inputs.fiducialTz[i] = fiducial.tz();
            inputs.fiducialQx[i] = fiducial.qx();
            inputs.fiducialQy[i] = fiducial.qy();
            inputs.fiducialQz[i] = fiducial.qz();
            inputs.fiducialQw[i] = fiducial.qw();
        }
    }

    private double getEntry(double[] values, int index) {
        return values.length > index ? values[index] : 0.0;
    }

    private boolean updateConnectionState(double nowSeconds) {
        if (!heartbeatEntry.exists()) {
            return false;
        }

        double heartbeat = heartbeatEntry.getDouble(Double.NaN);
        if (Double.isNaN(heartbeat) || heartbeat <= 0.0) {
            return false;
        }

        if (Double.isNaN(lastHeartbeat) || Double.compare(heartbeat, lastHeartbeat) != 0) {
            lastHeartbeat = heartbeat;
            lastHeartbeatUpdateTimeSeconds = nowSeconds;
        }

        return (nowSeconds - lastHeartbeatUpdateTimeSeconds) <= VisionConstants.limelightHeartbeatTimeoutSecs;
    }

    private boolean isFresh(LimelightObservation observation, double nowSeconds) {
        return (nowSeconds - observation.timestampSeconds()) <= VisionConstants.maxObservationAgeSecs;
    }

    private boolean isNewTimestamp(double timestampSeconds, double lastTimestampSeconds) {
        return timestampSeconds > lastTimestampSeconds + 1e-6;
    }

    private void setRobotOrientation(Rotation3d gyroRotation3d, AngularVelocity3d gyroVelocityRadPerSec) {
        LimelightHelpers.SetRobotOrientation(
                limelightName,
                Units.radiansToDegrees(gyroRotation3d.getZ()),
                Units.radiansToDegrees(gyroVelocityRadPerSec.yawRadPerSec()),
                Units.radiansToDegrees(gyroRotation3d.getY()),
                Units.radiansToDegrees(gyroVelocityRadPerSec.pitchRadPerSec()),
                Units.radiansToDegrees(gyroRotation3d.getX()),
                Units.radiansToDegrees(gyroVelocityRadPerSec.rollRadPerSec()));
    }

    private boolean shouldRejectEstimate(LimelightObservation observation, boolean rejectOnZError) {
        return !observation.fiducialsValid()
                || observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
                || (rejectOnZError && Math.abs(observation.pose().getZ()) > VisionConstants.maxZError)
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();
    }

    private double getAverageTagAmbiguity(RawFiducialParseResult rawFiducials) {
        if (rawFiducials == null || !rawFiducials.valid() || rawFiducials.fiducials().length == 0) {
            return Double.POSITIVE_INFINITY;
        }

        return Arrays.stream(rawFiducials.fiducials())
                .mapToDouble(fiducial -> fiducial.ambiguity)
                .average()
                .orElse(Double.POSITIVE_INFINITY);
    }

    @Override
    public List<PoseEstimation> updateInputs(
            VisionIOInputs inputs,
            Rotation3d gyroRotation3d,
            AngularVelocity3d gyroVelocityRadPerSec) {
        double nowSeconds = Timer.getFPGATimestamp();
        setRobotOrientation(gyroRotation3d, gyroVelocityRadPerSec);

        inputs.connected = updateConnectionState(nowSeconds);
        inputs.heartbeat = lastHeartbeat;
        inputs.latestObservationTimestampSeconds = Seconds.of(0.0);
        inputs.observedPoses = new Pose3d[] {};
        inputs.acceptedPoses = new Pose3d[] {};
        inputs.rejectedPoses = new Pose3d[] {};
        setCameraExtrinsicInputs(inputs);
        clearFiducialInputs(inputs);

        LimelightHelpers.LimelightResults latestResults = LimelightHelpers.getLatestResults(limelightName);
        setFiducialInputs(inputs, getFiducialLogData(latestResults, nowSeconds));

        Optional<LimelightObservation> observation = getObservation(VisionConstants.LIMELIGHT_ESTIMATION_MODE);
        if (observation.isEmpty()) {
            return List.of();
        }

        LimelightObservation estimated = observation.get();
        inputs.latestObservationTimestampSeconds = Seconds.of(estimated.timestampSeconds());
        inputs.observedPoses = new Pose3d[] { estimated.pose() };

        if (!inputs.connected
                || !isFresh(estimated, nowSeconds)
                || !isNewTimestamp(estimated.timestampSeconds(), lastMeasurementTimestampSeconds)) {
            return List.of();
        }

        if (shouldRejectEstimate(estimated, true)) {
            inputs.rejectedPoses = new Pose3d[] { estimated.pose() };
            return List.of();
        }

        lastMeasurementTimestampSeconds = estimated.timestampSeconds();
        inputs.acceptedPoses = new Pose3d[] { estimated.pose() };
        return List.of(VisionIO.PoseEstimation.fromStatistics(
                config,
                estimated.pose(),
                Seconds.of(estimated.timestampSeconds()),
                estimated.ambiguity(),
                estimated.avgTagDist(),
                estimated.tagCount(),
                estimated.isMegaTag2()));
    }

    @Override
    public void setIMUMode(int imuMode) {
        LimelightHelpers.SetIMUMode(limelightName, imuMode);
    }
}
