package frc.robot.supersystems.shooter;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.supersystems.shooter.ShotSolverUtil.KinematicsInfo;
import frc.robot.supersystems.shooter.ShotSolverUtil.ShotSolution;

public class TurretShotSolverAnglemap implements Supplier<ShotSolution> {
    private static class Constants {
        private static final Distance TURRET_X_OFFSET = Inches.of(-6.5); // negative X is back
        private static final Distance TURRET_Y_OFFSET = Inches.of(6.875); // positive Y is left
        private static final Distance TURRET_Z_OFFSET = Inches.of(20.5); // positive Z is up

        // private static final Distance passingShotClearance = Inches.of(18.0);

        private static final Translation3d TURRET_OFFSET = new Translation3d(TURRET_X_OFFSET, TURRET_Y_OFFSET,
                TURRET_Z_OFFSET);

        public static final double FLYWHEEL_RADIUS_METERS = 0.050;
        public static final double SPIN_TRANSFER_EFFICIENCY = 0.78;
    }

    private static record ShotMeasurement(double distanceMeters, double pitchRadians, double rpm) {
    }

    static final class MonotoneCubicInterpolator {
        private final double[] xs;
        private final double[] ys;
        private final double[] tangents;

        MonotoneCubicInterpolator(List<Double> xValues, List<Double> yValues) {
            if (xValues.size() != yValues.size()) {
                throw new IllegalArgumentException("x and y lists must be the same size");
            }

            if (xValues.size() < 2) {
                throw new IllegalArgumentException("at least two samples are required");
            }

            xs = xValues.stream().mapToDouble(Double::doubleValue).toArray();
            ys = yValues.stream().mapToDouble(Double::doubleValue).toArray();

            for (int i = 1; i < xs.length; i++) {
                if (xs[i] <= xs[i - 1]) {
                    throw new IllegalArgumentException("sample distances must be strictly increasing");
                }
            }

            tangents = computeTangents(xs, ys);
        }

        double interpolate(double x) {
            if (x <= xs[0]) {
                return ys[0];
            }
            if (x >= xs[xs.length - 1]) {
                return ys[ys.length - 1];
            }

            int upperIndex = Arrays.binarySearch(xs, x);
            if (upperIndex >= 0) {
                return ys[upperIndex];
            }

            upperIndex = -upperIndex - 1;
            int lowerIndex = upperIndex - 1;
            double intervalWidth = xs[upperIndex] - xs[lowerIndex];
            double t = (x - xs[lowerIndex]) / intervalWidth;
            double t2 = t * t;
            double t3 = t2 * t;

            double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            double h10 = t3 - 2.0 * t2 + t;
            double h01 = -2.0 * t3 + 3.0 * t2;
            double h11 = t3 - t2;

            return h00 * ys[lowerIndex]
                    + h10 * intervalWidth * tangents[lowerIndex]
                    + h01 * ys[upperIndex]
                    + h11 * intervalWidth * tangents[upperIndex];
        }

        private static double[] computeTangents(double[] xValues, double[] yValues) {
            int sampleCount = xValues.length;
            if (sampleCount == 2) {
                double slope = (yValues[1] - yValues[0]) / (xValues[1] - xValues[0]);
                return new double[] { slope, slope };
            }

            double[] intervals = new double[sampleCount - 1];
            double[] secants = new double[sampleCount - 1];
            for (int i = 0; i < sampleCount - 1; i++) {
                intervals[i] = xValues[i + 1] - xValues[i];
                secants[i] = (yValues[i + 1] - yValues[i]) / intervals[i];
            }

            double[] result = new double[sampleCount];
            result[0] = computeEndpointTangent(intervals[0], intervals[1], secants[0], secants[1]);
            result[sampleCount - 1] = computeEndpointTangent(
                    intervals[sampleCount - 2],
                    intervals[sampleCount - 3],
                    secants[sampleCount - 2],
                    secants[sampleCount - 3]);

            for (int i = 1; i < sampleCount - 1; i++) {
                if (secants[i - 1] == 0.0 || secants[i] == 0.0 || Math.signum(secants[i - 1]) != Math.signum(secants[i])) {
                    result[i] = 0.0;
                    continue;
                }

                double weight1 = 2.0 * intervals[i] + intervals[i - 1];
                double weight2 = intervals[i] + 2.0 * intervals[i - 1];
                result[i] = (weight1 + weight2)
                        / ((weight1 / secants[i - 1]) + (weight2 / secants[i]));
            }

            return result;
        }

        private static double computeEndpointTangent(double thisInterval, double nextInterval, double thisSecant,
                double nextSecant) {
            double tangent = ((2.0 * thisInterval + nextInterval) * thisSecant - thisInterval * nextSecant)
                    / (thisInterval + nextInterval);

            if (Math.signum(tangent) != Math.signum(thisSecant)) {
                return 0.0;
            }

            if (Math.signum(thisSecant) != Math.signum(nextSecant)
                    && Math.abs(tangent) > 3.0 * Math.abs(thisSecant)) {
                return 3.0 * thisSecant;
            }

            return tangent;
        }
    }

    private final Supplier<Pose3d> targetSupplier;
    private final Supplier<KinematicsInfo> kinematicsSupplier;
    private final List<ShotMeasurement> measurements = new ArrayList<>();
    private final MonotoneCubicInterpolator distanceToPitch;
    private final MonotoneCubicInterpolator distanceToRpm;

    private void putMeasurement(Distance dist, double pitchDegrees, double rpm) {
        measurements.add(new ShotMeasurement(dist.in(Meter), Math.toRadians(pitchDegrees), rpm));
    }

    public TurretShotSolverAnglemap(Supplier<Pose3d> targetSupplier, Supplier<KinematicsInfo> kinematicsSupplier) {
        // putMeasurement(Feet.of(7), 20, 1300);
        // putMeasurement(Feet.of(8), 24.0, 1450);
        // putMeasurement(Feet.of(9), 28.0, 1485);
        // putMeasurement(Feet.of(10), 28.0, 1555);
        // putMeasurement(Feet.of(11), 28.0, 1630);
        // putMeasurement(Feet.of(13), 28.0, 1820);
        // putMeasurement(Feet.of(16), 28.0, 2060);

        // putMeasurement(Feet.of(25), 33, 2225);
        // putMeasurement(Feet.of(40), 33, 2800);

        putMeasurement(Feet.of(4), 8, 1500);
        putMeasurement(Feet.of(5), 15, 1500);
        putMeasurement(Feet.of(6), 19, 1500);
        putMeasurement(Feet.of(7), 23, 1500);
        putMeasurement(Feet.of(8), 26, 1600);
        putMeasurement(Feet.of(9), 22.5, 2200);
        putMeasurement(Feet.of(10), 23.5, 2200);
        putMeasurement(Feet.of(11), 30, 1900);
        putMeasurement(Feet.of(12), 31, 2000);
        putMeasurement(Feet.of(13), 33, 2100);
        putMeasurement(Feet.of(14), 34, 2160);

        measurements.sort(Comparator.comparingDouble(ShotMeasurement::distanceMeters));

        List<Double> distances = new ArrayList<>(measurements.size());
        List<Double> pitches = new ArrayList<>(measurements.size());
        List<Double> rpms = new ArrayList<>(measurements.size());
        for (ShotMeasurement measurement : measurements) {
            distances.add(measurement.distanceMeters());
            pitches.add(measurement.pitchRadians());
            rpms.add(measurement.rpm());
        }

        this.targetSupplier = targetSupplier;
        this.kinematicsSupplier = kinematicsSupplier;
        this.distanceToPitch = new MonotoneCubicInterpolator(distances, pitches);
        this.distanceToRpm = new MonotoneCubicInterpolator(distances, rpms);
    }

    private Pair<Double, Double> getRPMAndPitch(double distance) {
        return Pair.of(distanceToRpm.interpolate(distance), distanceToPitch.interpolate(distance));
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

        Pose3d turretPosition = kinematicsInfo.position()
                .transformBy(new Transform3d(Constants.TURRET_OFFSET, Rotation3d.kZero));
        Translation3d fieldRelativeTurretOffset = Constants.TURRET_OFFSET
                .rotateBy(kinematicsInfo.position().getRotation());
        Translation3d turretVelocity = new Translation3d(
                kinematicsInfo.speeds().vxMetersPerSecond
                        - (fieldRelativeTurretOffset.getY() * kinematicsInfo.speeds().omegaRadiansPerSecond),
                kinematicsInfo.speeds().vyMetersPerSecond
                        + (fieldRelativeTurretOffset.getX() * kinematicsInfo.speeds().omegaRadiansPerSecond),
                0.0);

        Translation2d turretTranslation = turretPosition.getTranslation().toTranslation2d();
        Translation2d compensatedTarget = target.getTranslation().toTranslation2d();

        // comment below to disable velocity compensation (SotM)
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
        Rotation2d turretYaw = shotVector.getAngle().minus(kinematicsInfo.position().getRotation().toRotation2d());

        return new ShotSolution(
                RPM.of(rpmAndPitch.getFirst()),
                Radians.of(pitchRadians),
                Radians.of(turretYaw.getRadians()));
    }
}
