package frc.robot.supersystems.shooter;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
            return new Translation3d(cornerA.interpolate(cornerB, 0.5));
        }

        private static Translation3d getPassingTargetForBump(
                Translation2d nearLeftCorner,
                Translation2d nearRightCorner,
                double bumpHeightMeters) {
            Translation2d nearEdgeCenter = nearLeftCorner.interpolate(nearRightCorner, 0.5);
            return new Translation3d(
                    nearEdgeCenter.getX(),
                    nearEdgeCenter.getY(),
                    bumpHeightMeters + 1);
        }

        private final Translation3d leftBump = getPassingTargetForBump(
                FieldConstants.LeftBump.nearLeftCorner,
                FieldConstants.LeftBump.nearRightCorner,
                FieldConstants.LeftBump.height);
        private final Translation3d rightBump = getPassingTargetForBump(
                FieldConstants.RightBump.nearLeftCorner,
                FieldConstants.RightBump.nearRightCorner,
                FieldConstants.RightBump.height);

        private boolean isInAllianceZone(Pose3d robotPose) {
            return switch (DriverStation.getAlliance().orElse(Alliance.Blue)) {
                case Blue -> robotPose.getX() < FieldConstants.LinesVertical.neutralZoneNear;
                case Red -> robotPose.getX() > FieldConstants.LinesVertical.neutralZoneFar;
            };
        }

        @Override
        public Pose3d get() {
            Pose3d robotPose = kinematicsSupplier.get().position;
            Translation3d target;

            if (isInAllianceZone(robotPose)) { // not in neutral zone
                target = FieldConstants.Hub.innerCenterPoint; // flipped
            } else if (robotPose.getX() < FieldConstants.LinesVertical.neutralZoneFar) { // in neutral zone
                target = (robotPose.getY() > FieldConstants.LinesHorizontal.center)
                        ? rightBump
                        : leftBump;
            } else { // in opponent zone
                target = (robotPose.getY() > FieldConstants.LinesHorizontal.center)
                        ? rightBump
                        : leftBump;
            }

            return new Pose3d(AllianceFlipUtil.apply(target), new Rotation3d());
        }
    }
}
