package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class ShooterFlywheelsIOReal implements ShooterFlywheelsIO {
    private static class Constants {
        static final int leftFlywheelMotorId = 35;
        static final int rightFlywheelMotorId = 36;

        static final Slot0Configs flywheelSlot0 = new Slot0Configs()
                .withKS(0.3)
                .withKV(0.138203)
                .withKP(0.35);

        static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true);

        static final TalonFXConfiguration flywheelMotorConfig = new TalonFXConfiguration()
                .withSlot0(flywheelSlot0)
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(currentLimits);
    }

    private final TalonFX leftFlywheel = new TalonFX(Constants.leftFlywheelMotorId, RobotConstants.canBus);
    private final TalonFX rightFlywheel = new TalonFX(Constants.rightFlywheelMotorId, RobotConstants.canBus);

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterFlywheelsIOReal() {
        leftFlywheel.getConfigurator().apply(Constants.flywheelMotorConfig);
        rightFlywheel.getConfigurator().apply(Constants.flywheelMotorConfig);

        rightFlywheel.setControl(new Follower(Constants.leftFlywheelMotorId, MotorAlignmentValue.Opposed));
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
        inputs.leftFlywheelRPMs = leftFlywheel.getVelocity().getValue().in(RPM);
        inputs.leftCurrentAmps = leftFlywheel.getTorqueCurrent().getValueAsDouble();
        inputs.leftAppliedVoltage = leftFlywheel.getMotorVoltage().getValue().in(Volts);
        inputs.leftPositionRads = leftFlywheel.getPosition().getValue().in(Radians);

        inputs.rightFlywheelRPMs = rightFlywheel.getVelocity().getValue().in(RPM);
        inputs.rightCurrentAmps = rightFlywheel.getTorqueCurrent().getValueAsDouble();
        inputs.leftAppliedVoltage = rightFlywheel.getMotorVoltage().getValue().in(Volts);
        inputs.leftPositionRads = rightFlywheel.getPosition().getValue().in(Radians);
    }
}
