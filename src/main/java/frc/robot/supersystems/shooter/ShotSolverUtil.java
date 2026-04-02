package frc.robot.supersystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public final class ShotSolverUtil {
    public static record KinematicsInfo(Pose3d position, ChassisSpeeds speeds) {
    };

    public static record ShotSolution(AngularVelocity rpm, Angle pitch, Angle turretYaw) {
    };

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

    public static class TargetSupplier implements Supplier<Pose3d> {
        Supplier<KinematicsInfo> kinematicsSupplier;

        public TargetSupplier(Supplier<KinematicsInfo> kinematicsSupplier) {
            this.kinematicsSupplier = kinematicsSupplier;
        }

        private static Translation3d getAimPosition(Translation2d cornerA, Translation2d cornerB) {
            return new Translation3d(cornerA.plus(cornerB).div(2));
        }

        @Override
        public Pose3d get() {
            Pose3d robotPose = kinematicsSupplier.get().position;
            Translation3d target;

            if (robotPose.getX() < FieldConstants.LinesVertical.neutralZoneNear) { // not in neutral zone
                target = FieldConstants.Hub.innerCenterPoint;
            } else if (robotPose.getX() < FieldConstants.LinesVertical.neutralZoneFar) { // in neutral zone
                target = (robotPose.getY() > FieldConstants.LinesHorizontal.center)
                        ? getAimPosition(FieldConstants.RightBump.farLeftCorner,
                                FieldConstants.RightBump.farRightCorner)
                        : getAimPosition(FieldConstants.LeftBump.farLeftCorner, FieldConstants.LeftBump.farRightCorner);
            } else { // in opponent zone
                target = (robotPose.getY() > FieldConstants.LinesHorizontal.center)
                        ? getAimPosition(FieldConstants.RightBump.oppFarLeftCorner,
                                FieldConstants.RightBump.oppFarLeftCorner)
                        : getAimPosition(FieldConstants.LeftBump.oppFarLeftCorner,
                                FieldConstants.LeftBump.oppFarRightCorner);
            }

            return new Pose3d(AllianceFlipUtil.apply(target), new Rotation3d());
        }
    }
}
