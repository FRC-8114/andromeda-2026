package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakeRollersIOReal implements IntakeRollersIO {
    private static final int motorID = 52;

    private static final double gearRatio = 1.0;

    static final TalonFXConfiguration rollerMotorCfg = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(gearRatio))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(80));

    private final TalonFX rollerMotor = new TalonFX(motorID, RobotConstants.canBus);

    private static final VoltageOut controlVoltage = new VoltageOut(0);

    public void runVolts(Voltage volts) {
        rollerMotor.setControl(controlVoltage.withOutput(volts));
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }

    @Override
    public void updateInputs(IntakeRollersInputs inputs) {
        inputs.velocityRPM = rollerMotor.getVelocity().getValue().in(RPM);
        inputs.appliedVoltageVolts = rollerMotor.getMotorVoltage().getValue().in(Volts);
        inputs.rollerPositionRads = rollerMotor.getPosition().getValue().in(Radians);
    }
}
