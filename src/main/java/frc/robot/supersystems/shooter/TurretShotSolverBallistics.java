package frc.robot.supersystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.Drive;
import frc.robot.supersystems.shooter.ShotSolverUtil.ShotSolution;

public class TurretShotSolverBallistics implements Supplier<ShotSolution> {
    private static class Constants {
        private static final Distance TURRET_X_OFFSET = Inches.of(-6.5);   // negative X is back
        private static final Distance TURRET_Y_OFFSET = Inches.of(6.875);  // positive Y is left
        private static final Distance TURRET_Z_OFFSET = Inches.of(20.5);   // positive Z is up

        private static final Translation3d TURRET_OFFSET =
                new Translation3d(TURRET_X_OFFSET, TURRET_Y_OFFSET, TURRET_Z_OFFSET);

        // Ballistic solve in SI units.
        private static final double G_METERS_PER_SEC2 = 9.806635;

        /*
         * Fitted from your earlier experimental data:
         *
         *   v(ft/s) = 0.00414 * RPM + 10.97
         *
         * Converted to m/s:
         *
         *   v(m/s) = 0.001261872 * RPM + 3.343656
         */
        private static final double SPEED_PER_RPM_MPS = 0.00414 * 0.3048;
        private static final double SPEED_OFFSET_MPS = 10.97 * 0.3048;

        /*
         * Practical limits.
         * The fit came from ~1500-2500 RPM data, so below 1500 the model is much less trustworthy.
         */
        private static final double MIN_RPM = 1500.0;
        private static final double MAX_RPM = 5000.0;

        private static final double MIN_PITCH_DEG = 8.0;
        private static final double MAX_PITCH_DEG = 35.0;
        private static final double PITCH_SEARCH_STEP_DEG = 0.25;

        private static final int LEAD_ITERATIONS = 3;
    }

    private final Supplier<Pose3d> targetSupplier;
    private final Supplier<KinematicsInfo> kinematicsSupplier;

    public static record KinematicsInfo(Pose3d position, ChassisSpeeds speeds) {}

    private static record ShotParameters(double rpm, double pitchRadians, double launchSpeedMps) {}

    public static class DriveKinematicsSupplier implements Supplier<KinematicsInfo> {
        private final Drive drive;

        public DriveKinematicsSupplier(Drive drive) {
            this.drive = drive;
        }

        @Override
        public KinematicsInfo get() {
            Pose2d robotPose = drive.getPose();
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                    drive.getChassisSpeeds(),
                    robotPose.getRotation());

            return new KinematicsInfo(new Pose3d(robotPose), fieldSpeeds);
        }
    }

    public TurretShotSolverBallistics(
            Supplier<Pose3d> targetSupplier,
            Supplier<KinematicsInfo> kinematicsSupplier) {
        this.targetSupplier = targetSupplier;
        this.kinematicsSupplier = kinematicsSupplier;
    }

    /**
     * deltaH = d * tan(theta) - g d^2 / (2 v^2 cos^2(theta))
     * v = sqrt(g d^2 / (2 cos^2(theta) * (d tan(theta) - deltaH)))
     */
    public static double requiredSpeedMetersPerSecond(
            double horizontalDistanceMeters,
            double angleDeg,
            Distance targetHeight) {
        double deltaH = targetHeight.in(Meters) - Constants.TURRET_Z_OFFSET.in(Meters);
        double theta = Math.toRadians(angleDeg);

        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double denom = 2.0 * cos * cos * (horizontalDistanceMeters * tan - deltaH);
        if (denom <= 0.0) {
            return Double.POSITIVE_INFINITY;
        }

        return Math.sqrt(
                Constants.G_METERS_PER_SEC2
                        * horizontalDistanceMeters
                        * horizontalDistanceMeters
                        / denom);
    }

    public static double speedToRpm(double speedMetersPerSecond) {
        return (speedMetersPerSecond - Constants.SPEED_OFFSET_MPS) / Constants.SPEED_PER_RPM_MPS;
    }

    public static double rpmToSpeed(double rpm) {
        return Constants.SPEED_PER_RPM_MPS * rpm + Constants.SPEED_OFFSET_MPS;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private ShotParameters solveShot(
            double horizontalDistanceMeters,
            Distance targetHeight) {
        double bestRpm = Double.POSITIVE_INFINITY;
        double bestPitchDeg = Constants.MAX_PITCH_DEG;
        double bestLaunchSpeed = Double.POSITIVE_INFINITY;

        for (double pitchDeg = Constants.MIN_PITCH_DEG;
                pitchDeg <= Constants.MAX_PITCH_DEG + 1e-9;
                pitchDeg += Constants.PITCH_SEARCH_STEP_DEG) {

            double launchSpeed = requiredSpeedMetersPerSecond(
                    horizontalDistanceMeters,
                    pitchDeg,
                    targetHeight);

            if (!Double.isFinite(launchSpeed)) {
                continue;
            }

            double rpm = speedToRpm(launchSpeed);
            rpm = clamp(rpm, Constants.MIN_RPM, Constants.MAX_RPM);

            if (rpm < bestRpm) {
                bestRpm = rpm;
                bestPitchDeg = pitchDeg;
                bestLaunchSpeed = launchSpeed;
            }
        }

        if (!Double.isFinite(bestRpm)) {
            double fallbackPitchDeg = Constants.MAX_PITCH_DEG;
            double fallbackLaunchSpeed = requiredSpeedMetersPerSecond(
                    horizontalDistanceMeters,
                    fallbackPitchDeg,
                    targetHeight);

            double fallbackRpm = clamp(
                    speedToRpm(fallbackLaunchSpeed),
                    Constants.MIN_RPM,
                    Constants.MAX_RPM);

            return new ShotParameters(
                    fallbackRpm,
                    Math.toRadians(fallbackPitchDeg),
                    fallbackLaunchSpeed);
        }

        return new ShotParameters(
                bestRpm,
                Math.toRadians(bestPitchDeg),
                bestLaunchSpeed);
    }

    private double estimateTimeOfFlight(double horizontalDistanceMeters, ShotParameters shot) {
        double horizontalSpeed = shot.launchSpeedMps() * Math.cos(shot.pitchRadians());
        if (horizontalSpeed <= 1e-6) {
            return 0.0;
        }
        return horizontalDistanceMeters / horizontalSpeed;
    }

    @Override
    public ShotSolution get() {
        Pose3d target = targetSupplier.get();
        KinematicsInfo kinematicsInfo = kinematicsSupplier.get();

        Pose3d turretPosition = kinematicsInfo.position
                .transformBy(new Transform3d(Constants.TURRET_OFFSET, Rotation3d.kZero));

        Translation3d fieldRelativeTurretOffset =
                Constants.TURRET_OFFSET.rotateBy(kinematicsInfo.position.getRotation());

        Translation3d turretVelocity = new Translation3d(
                kinematicsInfo.speeds.vxMetersPerSecond
                        - fieldRelativeTurretOffset.getY() * kinematicsInfo.speeds.omegaRadiansPerSecond,
                kinematicsInfo.speeds.vyMetersPerSecond
                        + fieldRelativeTurretOffset.getX() * kinematicsInfo.speeds.omegaRadiansPerSecond,
                0.0);

        Translation2d turretTranslation = turretPosition.getTranslation().toTranslation2d();
        Translation2d compensatedTarget = target.getTranslation().toTranslation2d();

        ShotParameters shot = null;

        for (int i = 0; i < Constants.LEAD_ITERATIONS; i++) {
            Translation2d relativeTarget = compensatedTarget.minus(turretTranslation);

            shot = solveShot(
                    relativeTarget.getNorm(),
                    target.getMeasureZ());

            double timeOfFlight = estimateTimeOfFlight(relativeTarget.getNorm(), shot);

            compensatedTarget = target.getTranslation()
                    .minus(new Translation3d(
                            turretVelocity.getX() * timeOfFlight,
                            turretVelocity.getY() * timeOfFlight,
                            0.0))
                    .toTranslation2d();
        }

        Translation2d shotVector = compensatedTarget.minus(turretTranslation);

        if (shot == null) {
            shot = solveShot(
                    shotVector.getNorm(),
                    target.getMeasureZ());
        }

        Rotation2d turretYaw = shotVector.getAngle()
                .minus(kinematicsInfo.position.getRotation().toRotation2d());

        return new ShotSolution(
                RPM.of(shot.rpm()),
                Radians.of(shot.pitchRadians()),
                Radians.of(turretYaw.getRadians()));
    }
}