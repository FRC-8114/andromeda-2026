package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim implements VisionIO {
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

    private static VisionSystemSim visionSim;
    private static double lastSimUpdateTimeSec = 0;
    private PoseEstimation latestSeedObservation;

    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private final CameraConfiguration config;
    private final Supplier<Pose2d> poseSupplier;

    public VisionIOPhotonVisionSim(CameraConfiguration config, Supplier<Pose2d> poseSupplier) {
        this.config = config;
        this.poseSupplier = poseSupplier;

        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.aprilTagLayout);
        }

        camera = new PhotonCamera(config.name());

        var cameraProperties = SimCameraProperties.PERFECT_90DEG();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, config.robotToCamera());

        poseEstimator = new PhotonPoseEstimator(
                VisionConstants.aprilTagLayout,
                config.robotToCamera());
    }

    public CameraConfiguration getConfiguration() {
        return config;
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

    @Override
    public Optional<PoseEstimation> sampleSeedPose() {
        if (latestSeedObservation == null) {
            return Optional.empty();
        }

        if ((Timer.getFPGATimestamp() - latestSeedObservation.timestamp().in(Seconds)) > VisionConstants.maxObservationAgeSecs) {
            return Optional.empty();
        }

        return Optional.of(latestSeedObservation);
    }

    @Override
    public List<PoseEstimation> updateInputs(
            VisionIOInputs inputs,
            Rotation3d gyroRotation3d,
            AngularVelocity3d gyroVelocityRadPerSec) {
        double now = Timer.getFPGATimestamp();
        if (now - lastSimUpdateTimeSec > 0.005) {
            visionSim.update(poseSupplier.get());
            lastSimUpdateTimeSec = now;
        }
        inputs.connected = true;
        inputs.heartbeat = now;
        inputs.latestObservationTimestampSeconds = Seconds.of(0.0);
        setCameraExtrinsicInputs(inputs);
        clearFiducialInputs(inputs);
        List<Pose3d> observedPoses = new ArrayList<>();
        List<Pose3d> acceptedPoses = new ArrayList<>();
        List<PoseEstimation> observations = new ArrayList<>();
        List<FiducialLogData> fiducialLogData = new ArrayList<>();
        PoseEstimation bestSeedObservation = null;

        var results = camera.getAllUnreadResults();
        for (var result : results) {
            Optional<EstimatedRobotPose> estimated = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimated.isEmpty()) {
                estimated = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            if (estimated.isEmpty()) {
                continue;
            }

            EstimatedRobotPose pose = estimated.get();
            int tagCount = pose.targetsUsed.size();
            if (tagCount == 0) {
                continue;
            }

            double avgDist = pose.targetsUsed.stream()
                    .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                    .average()
                    .orElse(0.0);

            double avgAmbiguity = pose.targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average()
                    .orElse(0.0);

            inputs.latestObservationTimestampSeconds = Seconds.of(
                    Math.max(inputs.latestObservationTimestampSeconds.in(Seconds), pose.timestampSeconds));
            observedPoses.add(pose.estimatedPose);
            acceptedPoses.add(pose.estimatedPose);

            String sampleId = getSampleId(pose.timestampSeconds);
            for (var target : pose.targetsUsed) {
                var cameraToTarget = target.getBestCameraToTarget();
                var quaternion = cameraToTarget.getRotation().getQuaternion();
                fiducialLogData.add(new FiducialLogData(
                        sampleId,
                        config.name(),
                        target.getFiducialId(),
                        cameraToTarget.getX(),
                        cameraToTarget.getY(),
                        cameraToTarget.getZ(),
                        quaternion.getX(),
                        quaternion.getY(),
                        quaternion.getZ(),
                        quaternion.getW()));
            }

            PoseEstimation observation = PoseEstimation.fromStatistics(
                    config,
                    pose.estimatedPose,
                    Seconds.of(pose.timestampSeconds),
                    avgAmbiguity,
                    avgDist,
                    tagCount,
                    false);
            observations.add(observation);

            if (bestSeedObservation == null || observation.ambiguity() < bestSeedObservation.ambiguity()) {
                bestSeedObservation = observation;
            }
        }

        if (bestSeedObservation != null) {
            latestSeedObservation = bestSeedObservation;
        }
        inputs.observedPoses = observedPoses.toArray(new Pose3d[0]);
        inputs.acceptedPoses = acceptedPoses.toArray(new Pose3d[0]);
        inputs.rejectedPoses = new Pose3d[] {};
        setFiducialInputs(inputs, fiducialLogData);
        return observations;
    }
}
