package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;
import frc.robot.subsystems.vision.VisionConstants.LimelightCameraConfiguration;
import frc.robot.subsystems.vision.VisionIO.PoseEstimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private record IOBundle(VisionIO io, VisionIOInputsAutoLogged inputs) {}
    private final List<IOBundle> ioList;
    private final Consumer<PoseEstimation> poseConsumer;
    private final Consumer<PoseEstimation> poseSeedConsumer;
    private final Supplier<Rotation3d> gyroRotationSupplier;
    private final Supplier<AngularVelocity3d> gyroVelocitySupplier;

    private final List<Pose3d> allPoses = new ArrayList<Pose3d>();

    public Vision(
            Consumer<PoseEstimation> consumer,
            Consumer<PoseEstimation> seedConsumer,
            Supplier<Rotation3d> gyroRotationSupplier,
            Supplier<AngularVelocity3d> gyroVelocitySupplier,
            List<VisionIO> visionIOs) {
        ioList = visionIOs.stream()
                .map(io -> new IOBundle(io, new VisionIOInputsAutoLogged()))
                .toList();

        poseConsumer = consumer;
        poseSeedConsumer = seedConsumer;
        this.gyroRotationSupplier = gyroRotationSupplier;
        this.gyroVelocitySupplier = gyroVelocitySupplier;
    }

    public static Vision fromCameraConstants(
            Consumer<PoseEstimation> consumer,
            Consumer<PoseEstimation> seedConsumer,
            Supplier<Rotation3d> gyroRotationSupplier,
            Supplier<AngularVelocity3d> gyroVelocitySupplier,
            Supplier<Pose2d> robotPoseSupplier) {
        List<VisionIO> visionIOs = new ArrayList<>();

        for (CameraConfiguration camera : VisionConstants.cameras) {
            if (camera instanceof LimelightCameraConfiguration limelightCamera) {
                switch (RobotConstants.getRobotMode()) {
                    case REAL -> visionIOs.add(new VisionIOLimelight(limelightCamera));
                    case SIMULATION -> visionIOs.add(new VisionIOPhotonVisionSim(limelightCamera, robotPoseSupplier));
                    case REPLAY -> {
                    }
                }
            } else {
                throw new IllegalArgumentException("Unknown camera configuration type: " + camera.getClass());
            }
        }

        return new Vision(consumer, seedConsumer, gyroRotationSupplier, gyroVelocitySupplier, visionIOs);
    }

    public void seedPoseFromVision() {
        getLowestAmbiguitySeedObservation().ifPresent(poseSeedConsumer);
    }

    private Optional<PoseEstimation> getLowestAmbiguitySeedObservation() {
        PoseEstimation bestSeed = null;

        for (var bundle : ioList) {
            Optional<PoseEstimation> candidate = bundle.io().sampleSeedPose();
            if (candidate.isPresent()
                    && (bestSeed == null || candidate.get().ambiguity() < bestSeed.ambiguity())) {
                bestSeed = candidate.get();
            }
        }

        return Optional.ofNullable(bestSeed);
    }

    private void acceptObservation(PoseEstimation observation) {
        allPoses.add(observation.pose());
        poseConsumer.accept(observation);
    }

    public void setIMUMode(int mode) {
        for (var bundle : ioList) {
            bundle.io().setIMUMode(mode);
        }
    }

    @Override
    public void periodic() {
        allPoses.clear();

        Rotation3d gyroRotation3d = gyroRotationSupplier.get();
        AngularVelocity3d gyroVelocityRadPerSec = gyroVelocitySupplier.get();

        for (var bundle : ioList) {
            List<PoseEstimation> observations = bundle.io().updateInputs(
                    bundle.inputs(),
                    gyroRotation3d,
                    gyroVelocityRadPerSec);
            Logger.processInputs("Vision/" + bundle.io().getConfiguration().name(), bundle.inputs());
            observations.forEach(this::acceptObservation);
        }

        if (RobotState.isDisabled()) {
            seedPoseFromVision();
        }

    }

}
