package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>
 * Device configuration and other behaviors not exposed by TunerConstants can be
 * customized here.
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    // Hardware objects
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0);

    // Timestamp inputs from Phoenix thread
    private final Queue<Double> timestampQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final OpenLoopMode defaultDriveOpenLoopMode;
    private final OpenLoopMode defaultTurnOpenLoopMode;
    private OpenLoopMode driveOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
    private OpenLoopMode turnOpenLoopMode = OpenLoopMode.CLOSED_LOOP;
    private double driveOpenLoopOutput = 0.0;
    private double turnOpenLoopOutput = 0.0;

    public ModuleIOTalonFX(
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.constants = constants;
        defaultDriveOpenLoopMode = constants.DriveMotorClosedLoopOutput == SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC
                ? OpenLoopMode.TORQUE_CURRENT
                : OpenLoopMode.VOLTAGE;
        defaultTurnOpenLoopMode = constants.SteerMotorClosedLoopOutput == SwerveModuleConstants.ClosedLoopOutputType.TorqueCurrentFOC
                ? OpenLoopMode.TORQUE_CURRENT
                : OpenLoopMode.VOLTAGE;
        driveTalon = new TalonFX(constants.DriveMotorId, TunerConstants.kCANBus);
        turnTalon = new TalonFX(constants.SteerMotorId, TunerConstants.kCANBus);
        cancoder = new CANcoder(constants.EncoderId, TunerConstants.kCANBus);

        // Configure drive motor
        var driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted = constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

        // Configure turn motor
        var turnConfig = new TalonFXConfiguration();
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = constants.SteerMotorGains;
        turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
        turnConfig.Feedback.FeedbackSensorSource = switch (constants.FeedbackSource) {
            case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
            case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
            case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
            default ->
                throw new RuntimeException(
                        "You have selected a turn feedback source that is not supported by the default implementation of ModuleIOTalonFX. Please check the AdvantageKit documentation for more information on alternative configurations: https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations");
        };
        turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity
                / 0.100;
        turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

        // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.EncoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = constants.EncoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Create timestamp queue
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        // Create turn status signals
        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turnTalon.getPosition();
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnPosition.clone());
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        // Configure periodic frames
        BaseStatusSignal.setUpdateFrequencyForAll(
                Drive.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);
        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        // Refresh all signals
        var driveStatus = BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts,
                driveCurrent);
        var turnStatus = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
        var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

        // Update drive inputs
        inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
        inputs.drivePosition = drivePosition.getValue();
        inputs.driveVelocity = driveVelocity.getValue();
        inputs.driveAppliedVoltage = driveAppliedVolts.getValue();
        inputs.driveCurrent = driveCurrent.getValue();
        inputs.driveOpenLoopMode = driveOpenLoopMode;
        inputs.driveOpenLoopOutput = driveOpenLoopOutput;

        // Update turn inputs
        inputs.turnConnected = turnConnectedDebounce.calculate(turnStatus.isOK());
        inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
        inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnAbsolutePosition.getValue().in(Radians));
        inputs.turnPosition = turnPosition.getValue();
        inputs.turnVelocity = turnVelocity.getValue();
        inputs.turnAppliedVoltage = turnAppliedVolts.getValue();
        inputs.turnCurrent = turnCurrent.getValue();
        inputs.turnOpenLoopMode = turnOpenLoopMode;
        inputs.turnOpenLoopOutput = turnOpenLoopOutput;

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
                .mapToDouble((Double value) -> Rotations.of(value).in(Radians))
                .toArray();
        inputs.odometryTurnPositions = turnPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromRotations(value))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }

    private void setDriveOpenLoopState(OpenLoopMode mode, double output) {
        driveOpenLoopMode = mode;
        driveOpenLoopOutput = output;
    }

    private void setTurnOpenLoopState(OpenLoopMode mode, double output) {
        turnOpenLoopMode = mode;
        turnOpenLoopOutput = output;
    }

    private void applyDriveOpenLoop(double output, OpenLoopMode mode) {
        setDriveOpenLoopState(mode, output);
        driveTalon.setControl(
                switch (mode) {
                    case VOLTAGE -> voltageRequest.withOutput(output);
                    case TORQUE_CURRENT -> torqueCurrentRequest.withOutput(output);
                    case CLOSED_LOOP -> throw new IllegalArgumentException("Drive open-loop mode must not be CLOSED_LOOP");
                });
    }

    private void applyTurnOpenLoop(double output, OpenLoopMode mode) {
        setTurnOpenLoopState(mode, output);
        turnTalon.setControl(
                switch (mode) {
                    case VOLTAGE -> voltageRequest.withOutput(output);
                    case TORQUE_CURRENT -> torqueCurrentRequest.withOutput(output);
                    case CLOSED_LOOP -> throw new IllegalArgumentException("Turn open-loop mode must not be CLOSED_LOOP");
                });
    }

    public void setDriveOpenLoop(double output) {
        applyDriveOpenLoop(output, defaultDriveOpenLoopMode);
    }

    @Override
    public void setDriveCharacterizationVoltage(Voltage volts) {
        applyDriveOpenLoop(volts.in(Volts), OpenLoopMode.VOLTAGE);
    }

    public void setTurnOpenLoop(double output) {
        applyTurnOpenLoop(output, defaultTurnOpenLoopMode);
    }

    @Override
    public void setTurnCharacterizationVoltage(Voltage volts) {
        applyTurnOpenLoop(volts.in(Volts), OpenLoopMode.VOLTAGE);
    }

    public void setDriveVelocity(AngularVelocity velocity) {
        setDriveOpenLoopState(OpenLoopMode.CLOSED_LOOP, 0.0);
        double velocityRotPerSec = velocity.in(Rotations.per(Second));
        driveTalon.setControl(
                switch (constants.DriveMotorClosedLoopOutput) {
                    case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
                    case TorqueCurrentFOC ->
                        velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
                });
    }

    public void setTurnPosition(Rotation2d rotation) {
        setTurnOpenLoopState(OpenLoopMode.CLOSED_LOOP, 0.0);
        turnTalon.setControl(
                switch (constants.SteerMotorClosedLoopOutput) {
                    case Voltage -> positionVoltageRequest.withPosition(rotation.getRotations());
                    case TorqueCurrentFOC ->
                        positionTorqueCurrentRequest.withPosition(rotation.getRotations());
                });
    }
}
