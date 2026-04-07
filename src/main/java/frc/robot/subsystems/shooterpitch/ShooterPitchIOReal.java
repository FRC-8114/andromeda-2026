package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.robot.RobotConstants;

public class ShooterPitchIOReal implements ShooterPitchIO {
    private static class Constants {
        public static final int turretPitchMotorId = 38;

        public static final double turretPitchRatio = 12.67;

        private static final Slot0Configs pitchMotorPIDs = new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKS(0.55)
                .withKV(0.032)
                .withKA(0.011)
                .withKP(120.78)
                .withKG(0.4)
                .withKI(0.0)
                .withKD(0.0);

        private static final MotionMagicConfigs pitchMotionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(0.5)
                .withMotionMagicCruiseVelocity(0.1);

        private static final FeedbackConfigs pitchFeedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(turretPitchRatio);

        private static final SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(ShooterPitch.Constants.MAX_ANGLE)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(ShooterPitch.Constants.MIN_ANGLE);

        public static final TalonFXConfiguration pitchMotorCfg = new TalonFXConfiguration()
                .withSlot0(pitchMotorPIDs)
                .withSoftwareLimitSwitch(limits)
                .withMotionMagic(pitchMotionMagicConfigs)
                .withFeedback(pitchFeedbackConfigs)
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40));
    }

    private final TalonFX turretPitchMotor = new TalonFX(Constants.turretPitchMotorId, RobotConstants.canBus);
    private final MotionMagicVoltage control = new MotionMagicVoltage(ShooterPitch.Constants.MIN_ANGLE);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VoltageOut homingVoltageControl = new VoltageOut(0).withIgnoreSoftwareLimits(true);
    private final StatusSignal<Current> currentSignal = turretPitchMotor.getTorqueCurrent();

    public ShooterPitchIOReal() {
        turretPitchMotor.getConfigurator().apply(Constants.pitchMotorCfg);
        turretPitchMotor.setPosition(ShooterPitch.Constants.MIN_ANGLE);
    }

    public void setTarget(Angle angle) {
        turretPitchMotor.setControl(control.withPosition(angle));
    }

    public void setVoltage(double volts) {
        turretPitchMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setHomingVoltage(double volts) {
        turretPitchMotor.setControl(homingVoltageControl.withOutput(volts));
    }

    @Override
    public boolean supportsHomingReseed() {
        return true;
    }

    @Override
    public void reseedPosition(Angle angle) {
        turretPitchMotor.setPosition(angle);
    }

    public void updateInputs(ShooterPitchInputs inputs) {
        inputs.pitchPosition = turretPitchMotor.getPosition().getValue().in(Radians);
        inputs.velocityRadsPerSec = turretPitchMotor.getVelocity().getValue().in(RadiansPerSecond);
        inputs.appliedVoltage = turretPitchMotor.getMotorVoltage().getValueAsDouble();
        inputs.appliedCurrentAmps = currentSignal.refresh().getValueAsDouble();
    }
}
