package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.vision.VisionConstants.CameraConfiguration;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected;
        public double heartbeat = 0.0;
        public Time latestObservationTimestampSeconds = Seconds.of(0.0);
        public Pose3d[] observedPoses = new Pose3d[] {};
        public Pose3d[] acceptedPoses = new Pose3d[] {};
        public Pose3d[] rejectedPoses = new Pose3d[] {};
    }

    public static record PoseEstimation(
            Pose3d pose,
            Time timestamp,
            double ambiguity,
            Matrix<N3, N1> stddev) {

        public static PoseEstimation seedFromStatistics(
                Pose3d pose,
                Time timestamp,
                double ambiguity) {
            return new PoseEstimation(
                    pose,
                    timestamp,
                    ambiguity,
                    new Matrix<>(VecBuilder.fill(1e-4, 1e-4, 1e-4)));
        }

        public static PoseEstimation fromStatistics(
                CameraConfiguration configuration,
                Pose3d pose,
                Time timestamp,
                double ambiguity,
                double avgTagDist,
                int tagCount,
                boolean isMegaTag2) {
            double stdDevFactor = Math.pow(avgTagDist, 2.0) / Math.max(tagCount, 1);
            double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
            double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

            if (isMegaTag2) {
                linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
                angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
            }

            linearStdDev *= configuration.stdDeviation();
            angularStdDev *= configuration.stdDeviation();

            return new PoseEstimation(
                    pose,
                    timestamp,
                    ambiguity,
                    new Matrix<>(VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)));
        }
    }

    CameraConfiguration getConfiguration();

    default void setIMUMode(int imuMode) {}

    Optional<PoseEstimation> sampleSeedPose();

    List<PoseEstimation> updateInputs(
            VisionIOInputs inputs,
            Rotation3d gyroRotation3d,
            AngularVelocity3d gyroVelocityRadPerSec);
}
