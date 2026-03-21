package frc.robot.supersystems.shooter;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.Drive;

public class TurretShotSolver implements Supplier<ShotSolution> {
    private static class Constants {
        private static final Distance TURRET_X_OFFSET = Inches.of(-6.5); // negative X is back
        private static final Distance TURRET_Y_OFFSET = Inches.of(6.875); // positive Y is left
        private static final Distance TURRET_Z_OFFSET = Inches.of(20.5); // positive Z is up

        private static final Distance passingShotClearance = Inches.of(18.0);

        private static final Translation3d TURRET_OFFSET = new Translation3d(TURRET_X_OFFSET, TURRET_Y_OFFSET,
                TURRET_Z_OFFSET);

        public static final double FLYWHEEL_RADIUS_METERS = 0.050;
        public static final double SPIN_TRANSFER_EFFICIENCY = 0.75;
    }

    private Supplier<Pose3d> targetSupplier;
    private Supplier<KinematicsInfo> kinematicsSupplier;
    private final InterpolatingMatrixTreeMap<Double, N2, N1> distanceToPitchAndRPM = new InterpolatingMatrixTreeMap<Double, N2, N1>();

    private void putMeasurement(Distance dist, double pitchDegrees, double rpm) {
        double pitchRad = Math.toRadians(pitchDegrees);
        distanceToPitchAndRPM.put(dist.in(Meter) + 0.1,
                new Matrix<N2, N1>(new SimpleMatrix(new double[] { pitchRad, rpm })));
    }

    public static record KinematicsInfo(Pose3d position, ChassisSpeeds speeds) {
    };

    public static class DriveKinematicsSupplier implements Supplier<KinematicsInfo> {
        private final Drive drive;

        public DriveKinematicsSupplier(Drive drive) {
            this.drive = drive;
        }

        @Override
        public KinematicsInfo get() {
            Pose2d robotPose = drive.getPose();
            ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());

            return new KinematicsInfo(new Pose3d(robotPose), fieldSpeeds);
        }
    }

    public TurretShotSolver(Supplier<Pose3d> targetSupplier, Supplier<KinematicsInfo> kinematicsSupplier) {
        putMeasurement(Feet.of(7), 20, 1300);
        putMeasurement(Feet.of(8), 24.0, 1450);
        putMeasurement(Feet.of(9), 28.0, 1485);
        putMeasurement(Feet.of(10), 28.0, 1555);
        putMeasurement(Feet.of(11), 28.0, 1630);
        putMeasurement(Feet.of(13), 28.0, 1820);
        putMeasurement(Feet.of(16), 28.0, 2060);

        putMeasurement(Feet.of(25), 33, 2225);
        putMeasurement(Feet.of(40), 33, 2800);

        this.targetSupplier = targetSupplier;
        this.kinematicsSupplier = kinematicsSupplier;
    }

    private Pair<Double, Double> getRPMAndPitch(double distance) {
        var mat = distanceToPitchAndRPM.get(distance).getData();

        return Pair.of(mat[1], mat[0]);
    }

    private double estimateTimeOfFlight(double horizontalDist) {
        var rpmAndPitch = getRPMAndPitch(horizontalDist);
        double rpm = rpmAndPitch.getFirst();
        double pitchRad = rpmAndPitch.getSecond();
        double exitVelocity = (rpm * 2.0 * Math.PI * Constants.FLYWHEEL_RADIUS_METERS / 60.0)
                * Constants.SPIN_TRANSFER_EFFICIENCY;

        return horizontalDist / (exitVelocity * Math.cos(pitchRad));
    }

    @Override
    public ShotSolution get() {
        Pose3d target = targetSupplier.get();
        KinematicsInfo kinematicsInfo = kinematicsSupplier.get();

        Pose3d turretPosition = kinematicsInfo.position
                .transformBy(new Transform3d(Constants.TURRET_OFFSET, Rotation3d.kZero));
        Translation3d fieldRelativeTurretOffset = Constants.TURRET_OFFSET.rotateBy(kinematicsInfo.position.getRotation());
        Translation3d turretVelocity = new Translation3d(
                kinematicsInfo.speeds.vxMetersPerSecond
                        - (fieldRelativeTurretOffset.getY() * kinematicsInfo.speeds.omegaRadiansPerSecond),
                kinematicsInfo.speeds.vyMetersPerSecond
                        + (fieldRelativeTurretOffset.getX() * kinematicsInfo.speeds.omegaRadiansPerSecond),
                0.0);

        Translation2d turretTranslation = turretPosition.getTranslation().toTranslation2d();
        Translation2d compensatedTarget = target.getTranslation().toTranslation2d();

        for (int i = 0; i < 3; i++) {
            Translation2d relativeTarget = compensatedTarget.minus(turretTranslation);
            double timeOfFlight = estimateTimeOfFlight(relativeTarget.getNorm());
            compensatedTarget = target.getTranslation()
                    .minus(turretVelocity.times(timeOfFlight))
                    .toTranslation2d();
        }

        Translation2d shotVector = compensatedTarget.minus(turretTranslation);
        Pair<Double, Double> rpmAndPitch = getRPMAndPitch(shotVector.getNorm());
        double pitchRadians = rpmAndPitch.getSecond();
        Rotation2d turretYaw = shotVector.getAngle().minus(kinematicsInfo.position.getRotation().toRotation2d());

        return new ShotSolution(
                RPM.of(rpmAndPitch.getFirst()),
                Radians.of(pitchRadians),
                Radians.of(turretYaw.getRadians()));

    }
}
