package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import limelight.networktables.AngularAcceleration3d;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightSettings.ImuMode;

public class Vision extends SubsystemBase {
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }

    @FunctionalInterface
    public interface PoseSeedConsumer {
        void accept(Pose2d pose);
    }

    public static record RobotStateSample(
            double timestampSeconds,
            Rotation3d currentRotation,
            AngularVelocity3d currentAngularVelocity) {
    }

    private static final VisionIO.RobotState defaultRobotState = new VisionIO.RobotState(
            0.0,
            new Rotation3d(),
            new AngularVelocity3d(
                    RadiansPerSecond.of(0.0),
                    RadiansPerSecond.of(0.0),
                    RadiansPerSecond.of(0.0)),
            new AngularAcceleration3d(
                    RadiansPerSecondPerSecond.of(0.0),
                    RadiansPerSecondPerSecond.of(0.0),
                    RadiansPerSecondPerSecond.of(0.0)));

    private final VisionIO[] visionIOs;
    private final VisionIOInputsAutoLogged[] latestInputs;
    private final Supplier<RobotStateSample> robotStateSupplier;
    private final VisionMeasurementConsumer visionMeasurementConsumer;
    private final PoseSeedConsumer poseSeedConsumer;
    private VisionIO.RobotState latestRobotState = defaultRobotState;
    private AngularVelocity3d lastRobotAngularVelocity = new AngularVelocity3d(
            RadiansPerSecond.of(0.0),
            RadiansPerSecond.of(0.0),
            RadiansPerSecond.of(0.0));
    private double lastRobotAngularVelocityTimestampSecs = Double.NaN;
    private boolean seedMethodSet = false;

    public static Vision fromCameraConstants(
            Supplier<RobotStateSample> robotStateSupplier,
            VisionMeasurementConsumer visionMeasurementConsumer,
            PoseSeedConsumer poseSeedConsumer) {
        VisionIO[] inputs = new VisionIO[VisionConstants.cameras.length];

        for (int i = 0; i < VisionConstants.cameras.length; i++) {
            inputs[i] = new VisionIOLimelight(VisionConstants.cameras[i]);
        }

        return new Vision(robotStateSupplier, visionMeasurementConsumer, poseSeedConsumer, inputs);
    }

    public Vision(VisionIO... inputs) {
        this(null, null, inputs);
    }

    public Vision(Supplier<RobotStateSample> robotStateSupplier, VisionIO... inputs) {
        this(robotStateSupplier, null, inputs);
    }

    public Vision(
            Supplier<RobotStateSample> robotStateSupplier,
            VisionMeasurementConsumer visionMeasurementConsumer,
            VisionIO... inputs) {
        this(robotStateSupplier, visionMeasurementConsumer, null, inputs);
    }

    public Vision(
            Supplier<RobotStateSample> robotStateSupplier,
            VisionMeasurementConsumer visionMeasurementConsumer,
            PoseSeedConsumer poseSeedConsumer,
            VisionIO... inputs) {
        this.robotStateSupplier = robotStateSupplier;
        this.visionMeasurementConsumer = visionMeasurementConsumer;
        this.poseSeedConsumer = poseSeedConsumer;
        visionIOs = inputs;
        latestInputs = new VisionIOInputsAutoLogged[inputs.length];

        setIMUMode(ImuMode.SyncInternalImu);

        for (int i = 0; i < inputs.length; i++) {
            latestInputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    public void setIMUMode(ImuMode imuMode) {
        for (VisionIO visionIO : visionIOs) {
            visionIO.setIMUMode(imuMode);
        }
    }

    public void setRobotState(Rotation3d currentRotation, AngularVelocity3d currentAngularVelocity) {
        setRobotState(Timer.getFPGATimestamp(), currentRotation, currentAngularVelocity);
    }

    public void setRobotState(double timestampSeconds, Rotation3d currentRotation,
            AngularVelocity3d currentAngularVelocity) {
        AngularAcceleration3d currentAngularAcceleration = defaultRobotState.currentAngularAcceleration();

        if (!Double.isNaN(lastRobotAngularVelocityTimestampSecs)) {
            double dtSecs = timestampSeconds - lastRobotAngularVelocityTimestampSecs;

            if (dtSecs > 1.0e-6) {
                currentAngularAcceleration = new AngularAcceleration3d(
                        calculateAngularAcceleration(currentAngularVelocity.roll, lastRobotAngularVelocity.roll,
                                dtSecs),
                        calculateAngularAcceleration(currentAngularVelocity.pitch, lastRobotAngularVelocity.pitch,
                                dtSecs),
                        calculateAngularAcceleration(currentAngularVelocity.yaw, lastRobotAngularVelocity.yaw, dtSecs));
            }
        }

        lastRobotAngularVelocity = currentAngularVelocity;
        lastRobotAngularVelocityTimestampSecs = timestampSeconds;
        latestRobotState = new VisionIO.RobotState(
                timestampSeconds,
                currentRotation,
                currentAngularVelocity,
                currentAngularAcceleration);
    }

    private static AngularAcceleration calculateAngularAcceleration(
            AngularVelocity currentAngularVelocity,
            AngularVelocity lastAngularVelocity,
            double dtSecs) {
        return currentAngularVelocity.minus(lastAngularVelocity).div(Seconds.of(dtSecs));
    }

    private void processPoseObservations(VisionIOInputsAutoLogged inputs) {
        if (visionMeasurementConsumer == null) {
            return;
        }

        for (VisionIO.PoseObservation observation : inputs.poseObservations) {
            if (!shouldUseObservation(observation)) {
                continue;
            }

            visionMeasurementConsumer.accept(
                    observation.pose().toPose2d(),
                    observation.timestamp(),
                    getVisionMeasurementStdDevs(observation));
        }
    }

    private static boolean shouldUseObservation(VisionIO.PoseObservation observation) {
        if (observation.tagCount() <= 0) {
            return false;
        }

        if (Timer.getFPGATimestamp() - observation.timestamp() > VisionConstants.maxObservationAgeSecs) {
            return false;
        }

        if (observation.averageTagDistance() <= 0.0
                || observation.averageTagDistance() > VisionConstants.maxObservationDistanceMeters) {
            return false;
        }

        if (Math.abs(observation.pose().getZ()) > VisionConstants.maxZError) {
            return false;
        }

        return observation.type() != VisionIO.PoseObservationType.MEGATAG_1
                || observation.tagCount() > 1
                || observation.ambiguity() <= VisionConstants.maxAmbiguity;
    }

    private static VisionIO.PoseObservation choosePoseSeedObservation(
            VisionIO.PoseObservation currentBest,
            VisionIOInputsAutoLogged inputs) {
        VisionIO.PoseObservation bestObservation = currentBest;

        for (VisionIO.PoseObservation observation : inputs.seedObservations) {
            if (!shouldUseObservation(observation)) {
                continue;
            }

            if (bestObservation == null
                    || observation.tagCount() > bestObservation.tagCount()
                    || (observation.tagCount() == bestObservation.tagCount()
                            && observation.ambiguity() < bestObservation.ambiguity())) {
                bestObservation = observation;
            }
        }

        return bestObservation;
    }

    private static Matrix<N3, N1> getVisionMeasurementStdDevs(VisionIO.PoseObservation observation) {
        double distanceSquared = observation.averageTagDistance() * observation.averageTagDistance();
        double stdDevFactor = distanceSquared / observation.tagCount();
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

        if (observation.type() == VisionIO.PoseObservationType.MEGATAG_2) {
            linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
            angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
        } else {
            double ambiguityScale = 1.0 + observation.ambiguity();
            linearStdDev *= ambiguityScale;
            angularStdDev *= ambiguityScale;
        }

        linearStdDev *= observation.stdDevFactor();
        angularStdDev *= observation.stdDevFactor();

        return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous() || (DriverStation.isTeleop() && DriverStation.isDisabled())) {
            // yaw shouldnt drift much during auto
            setIMUMode(ImuMode.SyncInternalImu);

            if (DriverStation.isEnabled()) {
                // we have seeded from Choreo
                seedMethodSet = true;
            }
        }

        if (robotStateSupplier != null) {
            RobotStateSample robotStateSample = robotStateSupplier.get();

            if (robotStateSample != null) {
                setRobotState(
                        robotStateSample.timestampSeconds(),
                        robotStateSample.currentRotation(),
                        robotStateSample.currentAngularVelocity());
            }
        }

        VisionIO.PoseObservation[] seedObservationHolder = new VisionIO.PoseObservation[1];

        for (int i = 0; i < latestInputs.length; i++) {
            VisionIO io = visionIOs[i];
            VisionIOInputsAutoLogged inputs = latestInputs[i];

            io.setRobotState(latestRobotState);
            io.updateInputs(inputs);
            Logger.processInputs("Vision/" + io.getName(), inputs);

            if (DriverStation.isTeleop() && !seedMethodSet && poseSeedConsumer != null) {
                seedObservationHolder[0] = choosePoseSeedObservation(seedObservationHolder[0], inputs);
            }
            processPoseObservations(inputs);
        }

        if (DriverStation.isTeleopEnabled()) {
            seedMethodSet = true;
        }
    }

    public void triggerReseed() {
        VisionIO.PoseObservation seedObservationHolder[] = new VisionIO.PoseObservation[1];

        for (int i = 0; i < latestInputs.length; i++) {
            seedObservationHolder[0] = choosePoseSeedObservation(seedObservationHolder[0], latestInputs[i]);
        }

        if (poseSeedConsumer != null && seedObservationHolder[0] != null) {
            poseSeedConsumer.accept(seedObservationHolder[0].pose().toPose2d());
        }
    }
}
