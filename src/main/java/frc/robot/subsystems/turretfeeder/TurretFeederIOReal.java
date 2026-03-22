package frc.robot.subsystems.turretfeeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class TurretFeederIOReal implements TurretFeederIO {
    private static class Constants {
        private static final int turretLoaderMotorId = 42;

        static final Slot0Configs pidConfig = new Slot0Configs()
                .withKS(14.764)
                .withKP(37.58)
                .withKD(0.0);

        static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(70)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(70)
                    .withSupplyCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(1.875))
                .withSlot0(pidConfig);
    }

    private final TalonFX laneMotor = new TalonFX(Constants.turretLoaderMotorId, RobotConstants.canBus);

    private final VelocityVoltage control = new VelocityVoltage(0);
    private final VoltageOut controlVoltage = new VoltageOut(0);
    private final TorqueCurrentFOC controlTorque = new TorqueCurrentFOC(0);
    private final DutyCycleOut controlDutyCycle = new DutyCycleOut(1);

    public TurretFeederIOReal() {
        laneMotor.getConfigurator().apply(Constants.motorConfig);
    }

    public void runVolts(Voltage volts) {
        laneMotor.setControl(controlVoltage.withOutput(volts));
    }

    public void runTorqueCurrent(Current torque) {
        laneMotor.setControl(controlTorque.withOutput(torque));
    }

    public void setVelocity(AngularVelocity velocity) {
        laneMotor.setControl(control.withVelocity(velocity));
    }

    public void runDutyCycle() {
        laneMotor.setControl(controlDutyCycle.withOutput(1));
    }

    public void stopMotor() {
        laneMotor.stopMotor();
    }

    public void updateInputs(TurretFeederInputs inputs) {
        inputs.appliedVoltage = laneMotor.getMotorVoltage().getValue();
        inputs.feederPosition = laneMotor.getPosition().getValue();
        inputs.feederVelocity = laneMotor.getVelocity().getValue();
        inputs.appliedTorqueCurrent = laneMotor.getTorqueCurrent().getValue();
    }
}
