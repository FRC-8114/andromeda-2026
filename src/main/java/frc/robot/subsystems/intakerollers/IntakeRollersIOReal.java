package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakeRollersIOReal implements IntakeRollersIO {
    private static class Constants {
        private static final int ROLLER_MOTOR_ID = 52;

        private static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
                .withKS(0)
                .withKV(0)
                .withKP(0);

        private static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withSupplyCurrentLimit(80);

        static final TalonFXConfiguration ROLLER_MOTOR_CONFIGURATION = new TalonFXConfiguration()
                .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
                .withSlot0(SLOT0_CONFIGS);
    }

    private final TalonFX rollerMotor = new TalonFX(Constants.ROLLER_MOTOR_ID, RobotConstants.canBus);

    private final VoltageOut openLoopVoltage = new VoltageOut(0);
    private final TorqueCurrentFOC openLoopCurrentFOC = new TorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC closedLoopVelocity = new VelocityTorqueCurrentFOC(0);

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Angle> angleSignal;
    private final StatusSignalCollection rollerSignals = new StatusSignalCollection();

    private AngularVelocity targetAngularVelocity = RPM.of(0);

    public IntakeRollersIOReal() {
        velocitySignal = rollerMotor.getVelocity();
        voltageSignal = rollerMotor.getMotorVoltage();
        motorTorqueCurrent = rollerMotor.getTorqueCurrent();
        angleSignal = rollerMotor.getPosition();

        rollerSignals.addSignals(velocitySignal, voltageSignal, motorTorqueCurrent, angleSignal);
        rollerSignals.setUpdateFrequencyForAll(50);

        rollerMotor.optimizeBusUtilization();
        rollerMotor.getConfigurator().apply(Constants.ROLLER_MOTOR_CONFIGURATION);
    }

    public void setTargetVelocity(AngularVelocity velocity) {
        targetAngularVelocity = velocity;

        rollerMotor.setControl(closedLoopVelocity.withVelocity(velocity));
    }

    public void runVolts(Voltage voltage) {
        rollerMotor.setControl(openLoopVoltage.withOutput(voltage));
    }

    public void runCurrent(Current current) {
        rollerMotor.setControl(openLoopCurrentFOC.withOutput(current));
    }

    public void stop() {
        rollerMotor.stopMotor();
    }

    public void updateInputs(IntakeRollersInputs inputs) {
        rollerSignals.refreshAll();

        inputs.targetVelocityRadPerSec.mut_replace(targetAngularVelocity);

        inputs.velocityRadPerSec.mut_replace(velocitySignal.getValue());
        inputs.appliedVoltageVolts.mut_replace(voltageSignal.getValue());
        inputs.positionRadians.mut_replace(angleSignal.getValue());
        inputs.currentAmps.mut_replace(motorTorqueCurrent.getValue());
    }
}
