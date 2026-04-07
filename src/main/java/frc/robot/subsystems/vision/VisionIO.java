package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import limelight.networktables.AngularAcceleration3d;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightSettings.ImuMode;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
        public PoseObservation[] poseObservations = new PoseObservation[0];
        public PoseObservation[] seedObservations = new PoseObservation[0];
        public int[] tagIds = new int[0];
    }

    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
    }

    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            double stdDevFactor,
            PoseObservationType type) {
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    public static record RobotState(
            double timestampSeconds,
            Rotation3d currentRotation,
            AngularVelocity3d currentAngularVelocity,
            AngularAcceleration3d currentAngularAcceleration) {
    }

    String getName();

    void setRobotState(RobotState robotState);
    void setIMUMode(ImuMode imuMode);
    void updateInputs(VisionIOInputs inputs);
}
