package frc.robot.subsystems.shooterflywheels;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
        static final int LEFT_FLYWHEEL_MOTOR_ID = 35;
        static final int RIGHT_FLYWHEEL_MOTOR_ID = 36;
        static final double SIGNAL_UPDATE_HZ = 50.0;

        static final double gearRatio = 0.83;

        static final Slot0Configs FLYWHEEL_SLOT_0 = new Slot0Configs()
                .withKS(0.35)
                .withKV(0.09)
                .withKP(0.2);

        static final CurrentLimitsConfigs CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(70)
                .withSupplyCurrentLimit(70);

        static final TalonFXConfiguration FLYWHEEL_LEFT_MOTOR_CONFIG = new TalonFXConfiguration()
                .withSlot0(FLYWHEEL_SLOT_0)
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(gearRatio))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(CURRENT_LIMITS);
        static final TalonFXConfiguration FLYWHEEL_RIGHT_MOTOR_CONFIG = new TalonFXConfiguration()
                .withSlot0(FLYWHEEL_SLOT_0)
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(gearRatio))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(CURRENT_LIMITS);

        // static final TalonFXConfiguration FLYWHEEL_RIGHT_MOTOR_CONFIG = FLYWHEEL_LEFT_MOTOR_CONFIG.clone()
        //         .withMotorOutput(new MotorOutputConfigs()
        //                 .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    private final TalonFX leftFlywheel = new TalonFX(Constants.LEFT_FLYWHEEL_MOTOR_ID, RobotConstants.canBus);
    private final TalonFX rightFlywheel = new TalonFX(Constants.RIGHT_FLYWHEEL_MOTOR_ID, RobotConstants.canBus);

    private final StatusSignal<AngularVelocity> leftVelocity = leftFlywheel.getVelocity();
    private final StatusSignal<Current> leftCurrent = leftFlywheel.getTorqueCurrent();
    private final StatusSignal<Voltage> leftVoltage = leftFlywheel.getMotorVoltage();
    private final StatusSignal<Angle> leftPosition = leftFlywheel.getPosition();
    private final StatusSignal<AngularVelocity> rightVelocity = rightFlywheel.getVelocity();
    private final StatusSignal<Current> rightCurrent = rightFlywheel.getTorqueCurrent();
    private final StatusSignal<Voltage> rightVoltage = rightFlywheel.getMotorVoltage();
    private final StatusSignal<Angle> rightPosition = rightFlywheel.getPosition();

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0);
    private final TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final StrictFollower follower = new StrictFollower(Constants.RIGHT_FLYWHEEL_MOTOR_ID);

    public ShooterFlywheelsIOReal() {
        leftFlywheel.getConfigurator().apply(Constants.FLYWHEEL_LEFT_MOTOR_CONFIG);
        rightFlywheel.getConfigurator().apply(Constants.FLYWHEEL_RIGHT_MOTOR_CONFIG);

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.SIGNAL_UPDATE_HZ,
                leftVelocity,
                leftCurrent,
                leftVoltage,
                leftPosition,
                rightVelocity,
                rightCurrent,
                rightVoltage,
                rightPosition);
        ParentDevice.optimizeBusUtilizationForAll(leftFlywheel, rightFlywheel);
    }

    public void setFlywheelVelocity(AngularVelocity velocity) {
        rightFlywheel.setControl(velocityControl.withVelocity(velocity));
        leftFlywheel.setControl(follower);
    }

    public void runVolts(Voltage volts) {
        rightFlywheel.setControl(voltageControl.withOutput(volts));
        leftFlywheel.setControl(follower);
    }

    public void runCurrent(double current) {
        rightFlywheel.setControl(torqueControl.withOutput(current));
        leftFlywheel.setControl(follower);
    }

    public void stopFlywheels() {
        rightFlywheel.stopMotor();
        leftFlywheel.setControl(follower);
    }

    public void updateInputs(ShooterInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftVelocity,
                leftCurrent,
                leftVoltage,
                leftPosition,
                rightVelocity,
                rightCurrent,
                rightVoltage,
                rightPosition);

        inputs.leftFlywheelVelocity = leftVelocity.getValue();
        inputs.leftCurrent = leftCurrent.getValue();
        inputs.leftAppliedVoltage = leftVoltage.getValue();
        inputs.leftPosition = leftPosition.getValue();

        inputs.rightFlywheelVelocity = rightVelocity.getValue();
        inputs.rightCurrent = rightCurrent.getValue();
        inputs.rightAppliedVoltage = rightVoltage.getValue();
        inputs.rightPosition = rightPosition.getValue();
    }
}
