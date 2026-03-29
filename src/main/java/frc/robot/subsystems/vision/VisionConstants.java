
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public enum LimelightPoseMode {
        MEGATAG1,
        MEGATAG2
    }

    public static final LimelightPoseMode LIMELIGHT_ESTIMATION_MODE = LimelightPoseMode.MEGATAG1;
    public static final int LIMELIGHT_IMU_MODE = 4; // Internal IMU + external assist
    public static final double limelightHeartbeatTimeoutSecs = 0.5;
    public static final double maxObservationAgeSecs = 0.5;

    public interface CameraConfiguration {
        String name();
        double stdDeviation();
        Transform3d robotToCamera();
    }

    public static class LimelightCameraConfiguration implements CameraConfiguration {
        private final String camera;
        private final double stdDeviation;
        private final Transform3d robotToCamera;

        public LimelightCameraConfiguration(String cameraName, double stdDeviation, Transform3d robotToCamera) {
            this.camera = cameraName;
            this.stdDeviation = stdDeviation;
            this.robotToCamera = robotToCamera;
        }

        @Override
        public double stdDeviation() {
            return stdDeviation;
        }

        @Override
        public String name() {
            return camera;
        }

        @Override
        public Transform3d robotToCamera() {
            return robotToCamera;
        }
    }

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    private static final Transform3d ROBOT_TO_CAMERA_BACKLEFT = new Transform3d(
            new Translation3d(Inches.of(-10.8), Inches.of(12.375), Inches.of(8.75)),
            new Rotation3d(Degrees.of(0), Degrees.of(27), Degrees.of(135)));

    // right from the perspective of the robot
    private static final Transform3d ROBOT_TO_CAMERA_BACKRIGHT = new Transform3d(
            new Translation3d(Inches.of(-10.75), Inches.of(-11.25), Inches.of(11.25)),
            new Rotation3d(Degrees.of(0), Degrees.of(37), Degrees.of(225)));

    public static final CameraConfiguration[] cameras = {
            new LimelightCameraConfiguration("limelight-br", 1.0, ROBOT_TO_CAMERA_BACKRIGHT),
            new LimelightCameraConfiguration("limelight-bl", 1.0, ROBOT_TO_CAMERA_BACKLEFT)
    };

    public static boolean USE_TAG_WHITELIST = false;
    public static final int[] TAG_WHITELIST = {23, 29};

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
