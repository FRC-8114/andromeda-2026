package frc.robot.subsystems.hopperlanes;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class HopperLanesIOReal implements HopperLanesIO {
    private static final int indexerMotorId = 41;

    static final Slot0Configs pidConfig = new Slot0Configs()
        .withKS(0.25436)
        .withKV(0.012945)
        .withKA(0.0035911)
        .withKP(0.15104);

    static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(12))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true))
        .withSlot0(pidConfig);

    private final TalonFX laneMotor = new TalonFX(indexerMotorId, RobotConstants.canBus);

    private final VelocityVoltage control = new VelocityVoltage(0).withEnableFOC(true);

    private final VoltageOut controlVoltage = new VoltageOut(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);

    public HopperLanesIOReal() {
        laneMotor.getConfigurator().apply(motorConfig);
    }

    public void runVolts(Voltage volts) {
        laneMotor.setControl(controlVoltage.withOutput(volts));
    }

    public void setVelocity(AngularVelocity velocity) {
        laneMotor.setControl(control.withVelocity(velocity));
    }

    public void stopMotor() {
        laneMotor.stopMotor();
    }

    public void updateInputs(HopperLanesInputs inputs) {
        inputs.appliedTorqueCurrent = laneMotor.getTorqueCurrent().getValue().in(Amps);
        inputs.appliedVoltageVolts = laneMotor.getMotorVoltage().getValue().in(Volts);
        inputs.motorPositionRads = laneMotor.getPosition().getValue().in(Radians);
        inputs.velocityRPM = laneMotor.getVelocity().getValue().in(RPM);
    }

    @Override
    public void runCurrent(double current) {
        laneMotor.setControl(currentControl.withOutput(current));
    }
}
