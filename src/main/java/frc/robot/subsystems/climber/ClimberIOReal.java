package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class ClimberIOReal implements ClimberIO {
    private static class Constants {
        public static final int climbMotorID = 60;

        private static final double gearRatio = 36.0;

        private static final Slot0Configs climbMotorPIDs = new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(30)
                .withKI(0)
                .withKD(3);
        private static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
                .withSensorToMechanismRatio(gearRatio);
        public static final TalonFXConfiguration climbMotorCfg = new TalonFXConfiguration()
                .withSlot0(climbMotorPIDs)
                .withFeedback(feedbackConfig)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(Rotations.of(ClimberConstants.deployRotations))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(Rotations.of(ClimberConstants.stowRotations)))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40)
                        .withSupplyCurrentLimitEnable(true));
    }

    private static final TalonFX climbMotor = new TalonFX(Constants.climbMotorID, RobotConstants.canBus);

    private static final PositionVoltage control = new PositionVoltage(0).withEnableFOC(true);
    private static final VoltageOut controlVoltage = new VoltageOut(0);

    public ClimberIOReal() {
        climbMotor.getConfigurator().apply(Constants.climbMotorCfg);

        climbMotor.setPosition(Rotations.of(0)); // assume climb starts down
    }

    public void runVolts(Voltage volts) {
        climbMotor.setControl(controlVoltage.withOutput(volts).withIgnoreSoftwareLimits(true));
    }

    public void setPosition(double absoluteDrumRotations) {
        climbMotor.setControl(control.withPosition(Rotations.of(absoluteDrumRotations)));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        inputs.positionRot = climbMotor.getPosition().getValue().in(Rotations);
        inputs.velocityRPM = climbMotor.getVelocity().getValue().in(RPM);
        inputs.appliedVoltageVolts = climbMotor.getMotorVoltage().getValue().in(Volts);
        inputs.currentAmps = climbMotor.getSupplyCurrent().getValue().in(Amps);
    }
}
