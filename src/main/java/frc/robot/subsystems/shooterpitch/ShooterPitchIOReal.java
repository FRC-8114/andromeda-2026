package frc.robot.subsystems.shooterpitch;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class ShooterPitchIOReal implements ShooterPitchIO {
    private static class Constants {
        static final int MOTOR_ID = 38;
        static final double GEAR_RATIO = 12.67;

        static final Slot0Configs SLOT0_CONFIG = new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKS(0.55)
            .withKV(0.032)
            .withKA(0.011)
            .withKP(120.78)
            .withKG(0.4)
            .withKI(0.0)
            .withKD(0.0);

        static final MotionMagicConfigs MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
            .withMotionMagicAcceleration(0.5)
            .withMotionMagicCruiseVelocity(0.1);

        static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
            .withSensorToMechanismRatio(GEAR_RATIO);

        static final SoftwareLimitSwitchConfigs SOFT_LIMIT_CONFIG = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ShooterPitch.Constants.MAX_ANGLE)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ShooterPitch.Constants.MIN_ANGLE);

        static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true);

        static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIG)
            .withMotionMagic(MOTION_MAGIC_CONFIG)
            .withFeedback(FEEDBACK_CONFIG)
            .withSoftwareLimitSwitch(SOFT_LIMIT_CONFIG)
            .withCurrentLimits(CURRENT_LIMITS_CONFIG);
    }

    private final TalonFX pitchMotor = new TalonFX(Constants.MOTOR_ID, RobotConstants.canBus);

    private final MotionMagicVoltage controlPosition  = new MotionMagicVoltage(ShooterPitch.Constants.MIN_ANGLE);
    private final VoltageOut          controlVoltage  = new VoltageOut(0);
    private final VoltageOut          controlHoming   = new VoltageOut(0).withIgnoreSoftwareLimits(true);
    private final TorqueCurrentFOC    controlCurrent  = new TorqueCurrentFOC(0);

    private final StatusSignal<Angle>           positionSignal = pitchMotor.getPosition();
    private final StatusSignal<AngularVelocity> velocitySignal = pitchMotor.getVelocity();
    private final StatusSignal<Voltage>         voltageSignal  = pitchMotor.getMotorVoltage();
    private final StatusSignal<Current>         currentSignal  = pitchMotor.getTorqueCurrent();
    private final StatusSignalCollection        pitchSignals   = new StatusSignalCollection();

    private Angle targetAngle = ShooterPitch.Constants.MIN_ANGLE;

    public ShooterPitchIOReal() {
        pitchMotor.getConfigurator().apply(Constants.MOTOR_CONFIG);
        pitchMotor.setPosition(ShooterPitch.Constants.MIN_ANGLE);

        pitchSignals.addSignals(positionSignal, velocitySignal, voltageSignal, currentSignal);
        pitchSignals.setUpdateFrequencyForAll(50);

        pitchMotor.optimizeBusUtilization();
    }

    @Override
    public void setTarget(Angle angle) {
        targetAngle = angle;
        pitchMotor.setControl(controlPosition.withPosition(angle));
    }

    @Override
    public void setVoltage(Voltage volts) {
        pitchMotor.setControl(controlVoltage.withOutput(volts));
    }

    @Override
    public void setHomingVoltage(Voltage volts) {
        pitchMotor.setControl(controlHoming.withOutput(volts));
    }

    @Override
    public void runCurrent(Current current) {
        pitchMotor.setControl(controlCurrent.withOutput(current));
    }

    @Override
    public void stop() {
        pitchMotor.stopMotor();
    }

    @Override
    public boolean supportsHomingReseed() {
        return false;
    }

    @Override
    public void reseedPosition(Angle angle) {
        pitchMotor.setPosition(angle);
    }

    @Override
    public void updateInputs(ShooterPitchInputs inputs) {
        pitchSignals.refreshAll();

        inputs.targetAngle.mut_replace(targetAngle);
        inputs.position.mut_replace(positionSignal.getValue());
        inputs.velocity.mut_replace(velocitySignal.getValue());
        inputs.appliedVoltage.mut_replace(voltageSignal.getValue());
        inputs.appliedCurrent.mut_replace(currentSignal.getValue());
    }
}
