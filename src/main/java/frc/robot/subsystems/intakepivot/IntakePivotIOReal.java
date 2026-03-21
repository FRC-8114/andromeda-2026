package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakePivotIOReal implements IntakePivotIO {
    private static final int motorID = 51;

    private static final double gearRatio = 11.8125;

    private static final Slot0Configs pidConfig = new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKS(1.5)
            .withKG(0.4)
            .withKV(0.1)
            .withKA(0.01)
            .withKP(9.78);

    private static final MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(10)
            .withMotionMagicAcceleration(45);

    private static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withSlot0(pidConfig)
            .withMotionMagic(mmConfig)
            .withSoftwareLimitSwitch(
                    new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withForwardSoftLimitThreshold(IntakePivot.stowAngle)
                            .withReverseSoftLimitEnable(true).withReverseSoftLimitThreshold(IntakePivot.deployAngle))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(gearRatio))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive));

    private static final TalonFX pivotMotor = new TalonFX(motorID, RobotConstants.canBus);

    private static final MotionMagicVoltage control = new MotionMagicVoltage(IntakePivot.stowAngle);
    private static final VoltageOut controlVoltage = new VoltageOut(0).withEnableFOC(true);

    public IntakePivotIOReal() {
        pivotMotor.getConfigurator().apply(motorConfig);

        pivotMotor.setPosition(IntakePivot.stowAngle);
    }

    public void setTarget(Angle angle) {
        pivotMotor.setControl(control.withPosition(angle));
    }

    public void runVolts(Voltage volts) {
        pivotMotor.setControl(controlVoltage.withOutput(volts));
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        inputs.appliedVoltageVolts = pivotMotor.getMotorVoltage().getValue().in(Volts);
        inputs.positionRads = pivotMotor.getPosition().getValue().in(Radians);
        inputs.velocityRPM = pivotMotor.getVelocity().getValue().in(RPM);
    }
}