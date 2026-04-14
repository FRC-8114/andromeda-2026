package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakePivotIOReal implements IntakePivotIO {
    private static class Constants {
        static final int MOTOR_ID = 51;
        static final int ENCODER_ID = 53;
        static final double GEAR_RATIO = 11.8125;
        static final double MAGNET_OFFSET = -0.144775390625;
    }

    private static final MagnetSensorConfigs MAGNET_SENSOR_CONFIG = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(Constants.MAGNET_OFFSET)
            .withAbsoluteSensorDiscontinuityPoint(0.67);

    private static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration()
            .withMagnetSensor(MAGNET_SENSOR_CONFIG);

    private static final Slot0Configs PID_CONFIG = new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKG(0.1)
            .withKS(0.5)
            .withKP(6)
            .withKD(0);

    private static final MotionMagicConfigs MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(10)
            .withMotionMagicAcceleration(45);

    private static final SoftwareLimitSwitchConfigs SOFT_LIMIT_CONFIG = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(IntakePivot.Constants.STOWED_ANGLE)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(IntakePivot.Constants.DEPLOYED_ANGLE);

    private static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withFeedbackRemoteSensorID(Constants.ENCODER_ID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(Constants.GEAR_RATIO)
            .withSensorToMechanismRatio(1.0);

    private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);

    private static final MotorOutputConfigs MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);

    private static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(PID_CONFIG)
            .withMotionMagic(MOTION_MAGIC_CONFIG)
            .withSoftwareLimitSwitch(SOFT_LIMIT_CONFIG)
            .withFeedback(FEEDBACK_CONFIG)
            .withCurrentLimits(CURRENT_LIMITS_CONFIG)
            .withMotorOutput(MOTOR_OUTPUT_CONFIG);

    private final CANcoder pivotEncoder = new CANcoder(Constants.ENCODER_ID, RobotConstants.canBus);
    private final TalonFX pivotMotor = new TalonFX(Constants.MOTOR_ID, RobotConstants.canBus);

    private final MotionMagicVoltage control = new MotionMagicVoltage(0);
    private final TorqueCurrentFOC controlCurrent = new TorqueCurrentFOC(0);
    private final VoltageOut controlVoltage = new VoltageOut(0).withEnableFOC(true);

    // private final StatusSignal<Angle> encoderPositionSignal;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignalCollection pivotSignals = new StatusSignalCollection();

    private Angle targetAngle = IntakePivot.Constants.STOWED_ANGLE;

    public IntakePivotIOReal() {
        pivotMotor.getConfigurator().apply(MOTOR_CONFIG);
        pivotEncoder.getConfigurator().apply(ENCODER_CONFIG);

        // encoderPositionSignal = pivotEncoder.getPosition();
        positionSignal = pivotMotor.getPosition();
        velocitySignal = pivotMotor.getVelocity();
        voltageSignal = pivotMotor.getMotorVoltage();
        currentSignal = pivotMotor.getTorqueCurrent();

        pivotSignals.addSignals(positionSignal, velocitySignal, voltageSignal, currentSignal);
        pivotSignals.setUpdateFrequencyForAll(50);

        ParentDevice.optimizeBusUtilizationForAll(pivotEncoder, pivotMotor);

        Angle absolutePosition = pivotEncoder.getAbsolutePosition().refresh().getValue();
        pivotEncoder.setPosition(absolutePosition);
        pivotMotor.setPosition(absolutePosition);
    }

    @Override
    public void setTarget(Angle angle) {
        targetAngle = angle;
        pivotMotor.setControl(control.withPosition(angle));
    }

    @Override
    public void setTargetWithFeedForward(Angle angle, Voltage feedforward) {
        targetAngle = angle;
        pivotMotor.setControl(control.withPosition(angle).withFeedForward(feedforward));
    }

    @Override
    public void runVolts(Voltage volts) {
        pivotMotor.setControl(controlVoltage.withOutput(volts));
    }

    @Override
    public void runCurrent(Current current) {
        pivotMotor.setControl(controlCurrent.withOutput(current));
    }

    @Override
    public void stop() {
        pivotMotor.stopMotor();
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        pivotSignals.refreshAll();

        inputs.targetAngleRadians.mut_replace(targetAngle);
        inputs.positionRadians.mut_replace(positionSignal.getValue());
        inputs.velocityRadPerSec.mut_replace(velocitySignal.getValue());
        inputs.voltageVolts.mut_replace(voltageSignal.getValue());
        inputs.currentAmps.mut_replace(currentSignal.getValue());
    }
}
