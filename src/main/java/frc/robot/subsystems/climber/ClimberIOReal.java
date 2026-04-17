package frc.robot.subsystems.climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
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
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ClimberConstants.deployRotations)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ClimberConstants.stowRotations))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(80)
                        .withSupplyCurrentLimit(40));
    }

    private static final TalonFX climbMotor = new TalonFX(Constants.climbMotorID, RobotConstants.canBus);

    private final StatusSignal<Angle> positionSignal;    
    private final StatusSignal<AngularVelocity> drumVelocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> torqueCurrentSignal;
    private final StatusSignalCollection climberSignals = new StatusSignalCollection();

    private static final PositionVoltage control = new PositionVoltage(0).withEnableFOC(true);
    private static final VoltageOut controlVoltage = new VoltageOut(0);

    public ClimberIOReal() {
        climbMotor.getConfigurator().apply(Constants.climbMotorCfg);

        positionSignal = climbMotor.getPosition();
        drumVelocitySignal = climbMotor.getVelocity();
        voltageSignal = climbMotor.getMotorVoltage();
        torqueCurrentSignal = climbMotor.getTorqueCurrent();
        climberSignals.addSignals(
            positionSignal,
            drumVelocitySignal,
            voltageSignal,
            torqueCurrentSignal
        );
        climberSignals.setUpdateFrequencyForAll(50);
        ParentDevice.optimizeBusUtilizationForAll(climbMotor);

        climbMotor.setPosition(ClimberConstants.stowRotations); // assume climb starts down
    }

    public void runVolts(Voltage volts) {
        climbMotor.setControl(controlVoltage.withOutput(volts).withIgnoreSoftwareLimits(true));
    }

    public void setPosition(Angle position) {
        climbMotor.setControl(control.withPosition(position));
    }

    public void updateInputs(ClimberIOInputs inputs) {
        climberSignals.refreshAll();

        inputs.drumPosition.mut_replace(positionSignal.getValue());
        inputs.drumVelocity.mut_replace(drumVelocitySignal.getValue());
        inputs.appliedVoltage.mut_replace(voltageSignal.getValue());
        inputs.torqueCurrent.mut_replace(torqueCurrentSignal.getValue());
    }
}
