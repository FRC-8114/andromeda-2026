// Copyright (c) 2026 FRC Team 8114
// Copyright (c) 2021-2026 Littleton Robotics
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.RobotMode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.AngularVelocity3d;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private class Constants {
        public static final double HOLONOMIC_DRIVE_KP = 5.0;
        public static final double HOLONOMIC_DRIVE_KI = 0.0;
        public static final double HOLONOMIC_DRIVE_KD = 0.0;

        public static final double HOLONOMIC_TURN_KP = 10.0;
        public static final double HOLONOMIC_TURN_KI = 0.0;
        public static final double HOLONOMIC_TURN_KD = 0.0;
    }

    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    private final PIDController xController = new PIDController(Constants.HOLONOMIC_DRIVE_KP,
            Constants.HOLONOMIC_DRIVE_KI,
            Constants.HOLONOMIC_DRIVE_KD);
    private final PIDController yController = new PIDController(Constants.HOLONOMIC_DRIVE_KP,
            Constants.HOLONOMIC_DRIVE_KI,
            Constants.HOLONOMIC_DRIVE_KD);
    private final PIDController headingController = new PIDController(Constants.HOLONOMIC_TURN_KP,
            Constants.HOLONOMIC_TURN_KI, Constants.HOLONOMIC_TURN_KD);

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine translationSysId;
    private final SysIdRoutine steerSysId;
    private final SysIdRoutine rotationSysId;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private Rotation2d gyroYawOffset = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, Pose2d.kZero);

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure trajectory heading controller
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        // Configure drive translation SysId
        translationSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(7).per(Second),
                        Volts.of(20),
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdTranslationState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)),
                        (log) -> logTranslationSysIdTelemetry(),
                        this));

        // Configure steer SysId
        steerSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdSteerState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runSteerCharacterization(voltage.in(Volts)),
                        (log) -> logSteerSysIdTelemetry(),
                        this));

        // Configure rotational SysId.
        // The "voltage" value from SysId is intentionally interpreted as rotational
        // rate
        // request in rad/s to match CTRE's swerve rotation characterization workflow.
        rotationSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(7).per(Second),
                        Volts.of(40),
                        null,
                        (state) -> Logger.recordOutput("Drive/SysIdRotationState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runRotationCharacterization(voltage.in(Volts)),
                        (log) -> logRotationSysIdTelemetry(),
                        this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], getFieldGyroYaw(), modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && RobotConstants.getRobotMode() != RobotMode.SIMULATION);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Runs steer characterization by applying open-loop output to turn motors. */
    public void runSteerCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runSteerCharacterization(output);
        }
    }

    /**
     * Runs rotational characterization.
     * The request value is treated as desired rotational rate in rad/s.
     */
    public void runRotationCharacterization(double rotationalRateRadPerSec) {
        Logger.recordOutput(
                "Drive/RotationCharacterization/RequestedRateRadPerSec",
                rotationalRateRadPerSec);

        Translation2d[] moduleTranslations = getModuleTranslations();
        for (int i = 0; i < 4; i++) {
            double speedMps = rotationalRateRadPerSec * moduleTranslations[i].getNorm();
            double driveVolts = speedMps / getMaxLinearSpeedMetersPerSec() * 12.0;
            Rotation2d angle = moduleTranslations[i].getAngle().plus(Rotation2d.kCCW_90deg);
            modules[i].runCharacterization(driveVolts, angle);
        }
    }

    private void logTranslationSysIdTelemetry() {
        Logger.recordOutput("Drive/SysIdTranslation/DriveAppliedVolts", getDriveAppliedVolts());
        Logger.recordOutput("Drive/SysIdTranslation/DrivePositionMeters", getDrivePositionsMeters());
        Logger.recordOutput("Drive/SysIdTranslation/DriveVelocityMetersPerSec", getDriveVelocitiesMetersPerSec());
        Logger.recordOutput("Drive/SysIdTranslation/DriveCurrentAmps", getDriveCurrentsAmps());
    }

    private void logSteerSysIdTelemetry() {
        Logger.recordOutput("Drive/SysIdSteer/TurnAppliedVolts", getTurnAppliedVolts());
        Logger.recordOutput("Drive/SysIdSteer/TurnPositionRad", getTurnPositionsRad());
        Logger.recordOutput("Drive/SysIdSteer/TurnVelocityRadPerSec", getTurnVelocitiesRadPerSec());
        Logger.recordOutput("Drive/SysIdSteer/TurnCurrentAmps", getTurnCurrentsAmps());
    }

    private void logRotationSysIdTelemetry() {
        Logger.recordOutput("Drive/SysIdRotation/DriveAppliedVolts", getDriveAppliedVolts());
        Logger.recordOutput("Drive/SysIdRotation/DriveCurrentAmps", getDriveCurrentsAmps());
        Logger.recordOutput("Drive/SysIdRotation/YawPositionRad", getRawGyroYaw().getRadians());
        Logger.recordOutput("Drive/SysIdRotation/YawVelocityRadPerSec", getRotationCharacterizationYawVelocityRadPerSec());
    }

    private double[] getDriveAppliedVolts() {
        double[] driveAppliedVolts = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            driveAppliedVolts[i] = modules[i].getDriveAppliedVolts();
        }
        return driveAppliedVolts;
    }

    private double[] getDriveCurrentsAmps() {
        double[] driveCurrentsAmps = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            driveCurrentsAmps[i] = modules[i].getDriveCurrentAmps();
        }
        return driveCurrentsAmps;
    }

    private double[] getDrivePositionsMeters() {
        double[] drivePositionsMeters = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            drivePositionsMeters[i] = modules[i].getPositionMeters();
        }
        return drivePositionsMeters;
    }

    private double[] getDriveVelocitiesMetersPerSec() {
        double[] driveVelocitiesMetersPerSec = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            driveVelocitiesMetersPerSec[i] = modules[i].getVelocityMetersPerSec();
        }
        return driveVelocitiesMetersPerSec;
    }

    private double[] getTurnAppliedVolts() {
        double[] turnAppliedVolts = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            turnAppliedVolts[i] = modules[i].getTurnAppliedVolts();
        }
        return turnAppliedVolts;
    }

    private double[] getTurnPositionsRad() {
        double[] turnPositionsRad = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            turnPositionsRad[i] = modules[i].getTurnPositionRad();
        }
        return turnPositionsRad;
    }

    private double[] getTurnVelocitiesRadPerSec() {
        double[] turnVelocitiesRadPerSec = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            turnVelocitiesRadPerSec[i] = modules[i].getTurnVelocityRadPerSec();
        }
        return turnVelocitiesRadPerSec;
    }

    private double[] getTurnCurrentsAmps() {
        double[] turnCurrentsAmps = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            turnCurrentsAmps[i] = modules[i].getTurnCurrentAmps();
        }
        return turnCurrentsAmps;
    }

    private double getRotationCharacterizationYawVelocityRadPerSec() {
        if (gyroInputs.connected) {
            return gyroInputs.yawVelocityRadPerSec.in(RadiansPerSecond);
        }
        return getChassisSpeeds().omegaRadiansPerSecond;
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(translationSysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(translationSysId.dynamic(direction));
    }

    /**
     * Returns a command to run a steer quasistatic test in the specified direction.
     */
    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runSteerCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(steerSysId.quasistatic(direction));
    }

    /** Returns a command to run a steer dynamic test in the specified direction. */
    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runSteerCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(steerSysId.dynamic(direction));
    }

    /**
     * Returns a command to run a rotational quasistatic test in the specified
     * direction.
     * This dataset can be used for MOI estimation.
     */
    public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runRotationCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(rotationSysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a rotational dynamic test in the specified
     * direction.
     * This dataset can be used for MOI estimation.
     */
    public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runRotationCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(rotationSysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Returns the raw gyro yaw, unaffected by vision corrections. */
    public Rotation2d getRawGyroYaw() {
        return rawGyroRotation;
    }

    /** Returns the field-relative gyro yaw after applying any seeded offset. */
    public Rotation2d getFieldGyroYaw() {
        return rawGyroRotation.plus(gyroYawOffset);
    }

    /**
     * Returns the full 3D gyro rotation (roll, pitch, yaw), unaffected by vision
     * corrections.
     */
    public Rotation3d getRawGyroRotation3d() {
        return new Rotation3d(
                gyroInputs.rollPosition.getRadians(),
                gyroInputs.pitchPosition.getRadians(),
                rawGyroRotation.getRadians());
    }

    /**
     * Returns the full 3D gyro rotation with field-relative yaw for vision
     * consumers.
     */
    public Rotation3d getFieldGyroRotation3d() {
        return new Rotation3d(
                gyroInputs.rollPosition.getRadians(),
                gyroInputs.pitchPosition.getRadians(),
                getFieldGyroYaw().getRadians());
    }

    /**
     * Returns the full 3D gyro angular velocity (roll, pitch, yaw rates) in radians
     * per second.
     */
    public AngularVelocity3d getRawGyroVelocityRadPerSec() {
        return new AngularVelocity3d(
                gyroInputs.rollVelocityRadPerSec.in(RadiansPerSecond),
                gyroInputs.pitchVelocityRadPerSec.in(RadiansPerSecond),
                gyroInputs.yawVelocityRadPerSec.in(RadiansPerSecond));
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        gyroYawOffset = pose.getRotation().minus(rawGyroRotation);
        poseEstimator.resetPosition(getFieldGyroYaw(), getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
                new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading),
                pose.getRotation());

        Logger.recordOutput("Drive/TrajectorySetpointPose", sample.getPose());
        Logger.recordOutput("Drive/TrajectorySetpointSpeedsFieldRelative", sample.getChassisSpeeds());
        runVelocity(speeds);
    }
}
