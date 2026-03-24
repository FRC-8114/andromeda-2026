package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakePivotIOReal implements IntakePivotIO {
    private static final int motorID = 51;
    private static final int encoderID = 52;

    private static final double gearRatio = 11.8125;

    private final CANcoder pivotEncoder = new CANcoder(encoderID);

    private static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive) // TODO: make sure this is correct
            .withMagnetOffset(0)); // TODO: get magnet offset of cancoder 

    private static final Slot0Configs pidConfig = new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKS(7)
            .withKG(25)
            .withKP(1200)
            .withKD(0.7);

    private static final MotionMagicConfigs mmConfig = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(10)
            .withMotionMagicAcceleration(45);

    private static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withSlot0(pidConfig)
            .withMotionMagic(mmConfig)
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(IntakePivot.stowAngle)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(IntakePivot.deployAngle))
            .withFeedback(new FeedbackConfigs()
                .withFeedbackRemoteSensorID(encoderID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withRotorToSensorRatio(gearRatio)
                .withSensorToMechanismRatio(1.0))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40))
            .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive));

    private static final TalonFX pivotMotor = new TalonFX(motorID, RobotConstants.canBus);

    private static final MotionMagicTorqueCurrentFOC control = new MotionMagicTorqueCurrentFOC(IntakePivot.stowAngle);
    private static final VoltageOut controlVoltage = new VoltageOut(0).withEnableFOC(true);

    public IntakePivotIOReal() {
        pivotMotor.getConfigurator().apply(motorConfig);
        pivotEncoder.getConfigurator().apply(encoderConfig);
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