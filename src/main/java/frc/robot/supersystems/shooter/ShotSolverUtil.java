package frc.robot.supersystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.Drive;

public final class ShotSolverUtil {
    public static record KinematicsInfo(Pose3d position, ChassisSpeeds speeds) {};

    public static record ShotSolution(AngularVelocity rpm, Angle pitch, Angle turretYaw) {};

    public static class DriveKinematicsSupplier implements Supplier<KinematicsInfo> {
        private final Drive drive;

        public DriveKinematicsSupplier(Drive drive) {
            this.drive = drive;
        }

        @Override
        public KinematicsInfo get() {
            Pose2d robotPose = drive.getPose();
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(),
                    robotPose.getRotation());

            return new KinematicsInfo(new Pose3d(robotPose), fieldSpeeds);
        }
    }
}
