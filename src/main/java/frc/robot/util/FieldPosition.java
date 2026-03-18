package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.FieldConstants;

public final class FieldPosition {
    public enum Zone {
        ALLIANCE_ZONE,
        NEUTRAL_ZONE,
        OPPONENT_ZONE
    }

    private FieldPosition() {
    }

    public static Zone getZone(Pose2d pose) {
        return getZone(pose.getTranslation());
    }

    public static Zone getZone(Translation2d translation) {
        return getAllianceRelativeZone(AllianceFlipUtil.apply(translation));
    }

    public static Zone getZone(Translation3d translation) {
        return getAllianceRelativeZone(AllianceFlipUtil.apply(translation));
    }

    public static Zone getAllianceRelativeZone(Translation2d translation) {
        return getAllianceRelativeZone(translation.getX());
    }

    public static Zone getAllianceRelativeZone(Translation3d translation) {
        return getAllianceRelativeZone(translation.getX());
    }

    public static Zone getAllianceRelativeZone(double xPosition) {
        if (xPosition <= FieldConstants.LinesVertical.allianceZone) {
            return Zone.ALLIANCE_ZONE;
        }
        if (xPosition >= FieldConstants.LinesVertical.oppAllianceZone) {
            return Zone.OPPONENT_ZONE;
        }
        return Zone.NEUTRAL_ZONE;
    }
}
