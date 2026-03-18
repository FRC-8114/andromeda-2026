package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;

public final class AllianceFlipUtil {
    private static final Rotation2d HALF_TURN = new Rotation2d(Math.PI);

    private AllianceFlipUtil() {
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.fieldLength - x : x;
    }

    public static Translation2d apply(Translation2d translation) {
        return shouldFlip()
                ? new Translation2d(applyX(translation.getX()), translation.getY())
                : translation;
    }

    public static Translation3d apply(Translation3d translation) {
        return shouldFlip()
                ? new Translation3d(applyX(translation.getX()), translation.getY(), translation.getZ())
                : translation;
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? HALF_TURN.minus(rotation) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return shouldFlip() ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation())) : pose;
    }

    // Field-relative joystick commands need a 180-degree driver perspective shift on red.
    public static Rotation2d applyFieldRelative(Rotation2d heading) {
        return shouldFlip() ? heading.plus(HALF_TURN) : heading;
    }
}
