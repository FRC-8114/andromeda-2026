package frc.robot.subsystems.intakepivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.StatusSignal;
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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class IntakePivotIOReal implements IntakePivotIO {
    private static final int motorID = 51;
    private static final int encoderID = 53;

    private final CANcoder pivotEncoder = new CANcoder(encoderID, RobotConstants.canBus);

    private static final CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(-0.69677734375)); 

    private static final Slot0Configs pidConfig = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKS(10)
        .withKG(5)
        .withKP(600)
        .withKD(40);

    private static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        .withSlot0(pidConfig)
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(10)
            .withMotionMagicAcceleration(45))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(IntakePivotConstants.stowAngle)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(IntakePivotConstants.deployAngle))
        .withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(encoderID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withRotorToSensorRatio(IntakePivotConstants.gearRatio)
            .withSensorToMechanismRatio(1.0))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withSupplyCurrentLimit(40))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive));

    private static final TalonFX pivotMotor = new TalonFX(motorID, RobotConstants.canBus);

    private StatusSignal<Angle> position;
    private StatusSignal<AngularVelocity> velocity;
    private StatusSignal<Voltage> voltage;
    private StatusSignal<Current> torqueCurrent;

    private static final MotionMagicTorqueCurrentFOC control = new MotionMagicTorqueCurrentFOC(0);
    private static final VoltageOut controlVoltage = new VoltageOut(0).withEnableFOC(true);

    public IntakePivotIOReal() {
        pivotMotor.getConfigurator().apply(motorConfig);
        pivotEncoder.getConfigurator().apply(encoderConfig);

        position = pivotMotor.getPosition();
        velocity = pivotMotor.getVelocity();
        voltage = pivotMotor.getMotorVoltage();
        torqueCurrent = pivotMotor.getTorqueCurrent();
    }

    public void runVolts(Voltage volts) {
        pivotMotor.setControl(controlVoltage.withOutput(volts));
    }

    public void setTarget(Angle angle) {
        pivotMotor.setControl(control.withPosition(angle));
    }
    public void holdTargetDown(Angle angle) {
        pivotMotor.setControl(control.withPosition(angle).withFeedForward(Amps.of(25)));
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        inputs.positionDeg = position.getValue().in(Degrees);
        inputs.velocityRPM = velocity.getValue().in(RPM);
        inputs.appliedVoltageVolts = voltage.getValueAsDouble();
        inputs.appliedTorqueCurrent = torqueCurrent.getValueAsDouble();
    }
}