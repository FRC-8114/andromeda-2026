package frc.robot.subsystems.shooterflywheels;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class ShooterFlywheelsIOReal implements ShooterFlywheelsIO {
    private static class Constants {
        static final int LEFT_MOTOR_ID = 35;
        static final int RIGHT_MOTOR_ID = 36;
        static final double GEAR_RATIO = 0.83;

        static final Slot0Configs SLOT0_CONFIG = new Slot0Configs()
                .withKS(0.5)
                .withKV(0.09)
                .withKP(0.4);

        static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(70)
                .withSupplyCurrentLimit(70);

        static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
                .withSensorToMechanismRatio(GEAR_RATIO);

        static final TalonFXConfiguration LEFT_MOTOR_CONFIG = new TalonFXConfiguration()
                .withSlot0(SLOT0_CONFIG)
                .withFeedback(FEEDBACK_CONFIG)
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(CURRENT_LIMITS_CONFIG);

        static final TalonFXConfiguration RIGHT_MOTOR_CONFIG = new TalonFXConfiguration()
                .withSlot0(SLOT0_CONFIG)
                .withFeedback(FEEDBACK_CONFIG)
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(CURRENT_LIMITS_CONFIG);
    }

    private final TalonFX leftFlywheel = new TalonFX(Constants.LEFT_MOTOR_ID, RobotConstants.canBus);
    private final TalonFX rightFlywheel = new TalonFX(Constants.RIGHT_MOTOR_ID, RobotConstants.canBus);

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final StrictFollower follower = new StrictFollower(Constants.RIGHT_MOTOR_ID);

    private final StatusSignal<AngularVelocity> leftVelocitySignal = leftFlywheel.getVelocity();
    private final StatusSignal<Current> leftCurrentSignal = leftFlywheel.getTorqueCurrent();
    private final StatusSignal<Voltage> leftVoltageSignal = leftFlywheel.getMotorVoltage();
    private final StatusSignal<Angle> leftPositionSignal = leftFlywheel.getPosition();
    private final StatusSignal<AngularVelocity> rightVelocitySignal = rightFlywheel.getVelocity();
    private final StatusSignal<Current> rightCurrentSignal = rightFlywheel.getTorqueCurrent();
    private final StatusSignal<Voltage> rightVoltageSignal = rightFlywheel.getMotorVoltage();
    private final StatusSignal<Angle> rightPositionSignal = rightFlywheel.getPosition();
    private final StatusSignalCollection flywheelSignals = new StatusSignalCollection();

    private AngularVelocity targetVelocity = leftVelocitySignal.getValue();

    public ShooterFlywheelsIOReal() {
        leftFlywheel.getConfigurator().apply(Constants.LEFT_MOTOR_CONFIG);
        rightFlywheel.getConfigurator().apply(Constants.RIGHT_MOTOR_CONFIG);

        flywheelSignals.addSignals(
                leftVelocitySignal, leftCurrentSignal, leftVoltageSignal, leftPositionSignal,
                rightVelocitySignal, rightCurrentSignal, rightVoltageSignal, rightPositionSignal);
        flywheelSignals.setUpdateFrequencyForAll(50);

        ParentDevice.optimizeBusUtilizationForAll(leftFlywheel, rightFlywheel);
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        rightFlywheel.setControl(velocityControl.withVelocity(velocity));
        leftFlywheel.setControl(follower);
    }

    public void runVolts(Voltage volts) {
        rightFlywheel.setControl(voltageControl.withOutput(volts));
        leftFlywheel.setControl(follower);
    }

    public void runCurrent(Current current) {
        rightFlywheel.setControl(currentControl.withOutput(current));
        leftFlywheel.setControl(follower);
    }

    public void stop() {
        rightFlywheel.stopMotor();
        leftFlywheel.setControl(follower);
    }

    public void updateInputs(ShooterFlywheelsInputs inputs) {
        flywheelSignals.refreshAll();

        inputs.targetVelocity.mut_replace(targetVelocity);

        inputs.leftVelocity.mut_replace(leftVelocitySignal.getValue());
        inputs.leftCurrent.mut_replace(leftCurrentSignal.getValue());
        inputs.leftAppliedVoltage.mut_replace(leftVoltageSignal.getValue());
        inputs.leftPosition.mut_replace(leftPositionSignal.getValue());

        inputs.rightVelocity.mut_replace(rightVelocitySignal.getValue());
        inputs.rightCurrent.mut_replace(rightCurrentSignal.getValue());
        inputs.rightAppliedVoltage.mut_replace(rightVoltageSignal.getValue());
        inputs.rightPosition.mut_replace(rightPositionSignal.getValue());
    }
}
