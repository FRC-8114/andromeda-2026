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
        List<Pose3d> observedPoses = new ArrayList<>();
        List<Pose3d> acceptedPoses = new ArrayList<>();
        List<PoseEstimation> observations = new ArrayList<>();
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
        return observations;
    }
}
