package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class ShooterFlywheelsIOReal implements ShooterFlywheelsIO {
    private static class Constants {
        static final int LEFT_FLYWHEEL_MOTOR_ID = 35;
        static final int RIGHT_FLYWHEEL_MOTOR_ID = 36;
        static final double SIGNAL_UPDATE_HZ = 50.0;

        static final Slot0Configs FLYWHEEL_SLOT_0 = new Slot0Configs()
                .withKS(0.3)
                .withKV(0.138203)
                .withKP(0.35);

        static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true);

        static final TalonFXConfiguration FLYWHEEL_MOTOR_CONFIG = new TalonFXConfiguration()
                .withSlot0(FLYWHEEL_SLOT_0)
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(CURRENT_LIMITS);
    }

    private final TalonFX leftFlywheel = new TalonFX(Constants.LEFT_FLYWHEEL_MOTOR_ID, RobotConstants.canBus);
    private final TalonFX rightFlywheel = new TalonFX(Constants.RIGHT_FLYWHEEL_MOTOR_ID, RobotConstants.canBus);

    private final StatusSignal<AngularVelocity> leftVelocity = leftFlywheel.getVelocity();
    private final StatusSignal<Current> leftCurrent = leftFlywheel.getTorqueCurrent();
    private final StatusSignal<Voltage> leftVoltage = leftFlywheel.getMotorVoltage();
    private final StatusSignal<Angle> leftPosition = leftFlywheel.getPosition();
    private final StatusSignal<AngularVelocity> rightVelocity = rightFlywheel.getVelocity();
    private final StatusSignal<Current> rightCurrent = rightFlywheel.getTorqueCurrent();
    private final StatusSignal<Voltage> rightVoltage = rightFlywheel.getMotorVoltage();
    private final StatusSignal<Angle> rightPosition = rightFlywheel.getPosition();

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterFlywheelsIOReal() {
        leftFlywheel.getConfigurator().apply(Constants.FLYWHEEL_MOTOR_CONFIG);
        rightFlywheel.getConfigurator().apply(Constants.FLYWHEEL_MOTOR_CONFIG);

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_HZ,
                leftVelocity,
                leftCurrent,
                leftVoltage,
                leftPosition,
                rightVelocity,
                rightCurrent,
                rightVoltage,
                rightPosition);
        ParentDevice.optimizeBusUtilizationForAll(leftFlywheel, rightFlywheel);

        rightFlywheel.setControl(new Follower(Constants.LEFT_FLYWHEEL_MOTOR_ID, MotorAlignmentValue.Opposed));
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        leftFlywheel.setControl(velocityControl.withVelocity(velocity));
    }

    public void runVolts(Voltage volts) {
        leftFlywheel.setControl(voltageControl.withOutput(volts));
    }

    public void stopFlywheels() {
        leftFlywheel.stopMotor();
        rightFlywheel.stopMotor();
    }

    public void updateInputs(ShooterInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftVelocity,
                leftCurrent,
                leftVoltage,
                leftPosition,
                rightVelocity,
                rightCurrent,
                rightVoltage,
                rightPosition);

        inputs.leftFlywheelRPMs = leftVelocity.getValue().in(RPM);
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftAppliedVoltage = leftVoltage.getValue().in(Volts);
        inputs.leftPositionRads = leftPosition.getValue().in(Radians);

        inputs.rightFlywheelRPMs = rightVelocity.getValue().in(RPM);
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightAppliedVoltage = rightVoltage.getValue().in(Volts);
        inputs.rightPositionRads = rightPosition.getValue().in(Radians);
    }
}
