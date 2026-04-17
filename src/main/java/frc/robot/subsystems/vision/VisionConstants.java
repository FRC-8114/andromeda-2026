
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;
import limelight.networktables.LimelightSettings.ImuMode;

public class VisionConstants {
    public static final ImuMode LIMELIGHT_IMU_MODE = ImuMode.InternalImuExternalAssist;
    public static final ImuMode LIMELIGHT_SEED_IMU_MODE = ImuMode.SyncInternalImu;
    public static final double limelightHeartbeatTimeoutSecs = 0.5;
    public static final double maxObservationAgeSecs = 0.5;
    public static final double maxObservationDistanceMeters = 8.0;

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
    public static AprilTagFieldLayout aprilTagLayout = FieldConstants.fieldLayout;
    /*
     * Back Left:
     * Pitch: 28.10
     * Yaw:45
     * ignore -14.25 + 1.84
     * ignore -12.75 + 1.907
     * height from floor: 8.222
     * 
     * -10.968
     * -12.535
     */
    // limelight is right-positive
    private static final Transform3d ROBOT_TO_CAMERA_BACKLEFT = new Transform3d(
            new Translation3d(Inches.of(-10.968), Inches.of(-12.535), Inches.of(8.222)),
            new Rotation3d(Degrees.of(0), Degrees.of(28.10), Degrees.of(135)));

    /*
     * Back Right:
     * Pitch:28.1
     * Yaw:45
     * ignore 14.25 - 1.799
     * ignore -12.75 + 1.924
     * height from floor: 11.487
     * 
     * -10.951
     * 12.576
     * 
     */
    // right from the perspective of the robot
    private static final Transform3d ROBOT_TO_CAMERA_BACKRIGHT = new Transform3d(
            new Translation3d(Inches.of(-10.951), Inches.of(12.576), Inches.of(11.487)),
            new Rotation3d(Degrees.of(0), Degrees.of(28.1), Degrees.of(225)));

    public static final CameraConfiguration[] cameras = {
            new LimelightCameraConfiguration("limelight-br", 1.0, ROBOT_TO_CAMERA_BACKRIGHT),
            new LimelightCameraConfiguration("limelight-bl", 1.0, ROBOT_TO_CAMERA_BACKLEFT)
    };

    public static boolean USE_TAG_WHITELIST = false;
    public static final int[] TAG_WHITELIST = { 23, 29 };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.7; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
