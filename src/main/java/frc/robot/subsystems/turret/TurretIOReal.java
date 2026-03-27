package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class TurretIOReal implements TurretIO {
    private static class CANConfiguration {
        private static final int PIVOT_MOTOR_ID = 32;
        private static final int ENCODER_19T_ID = 33;
        private static final int ENCODER_21T_ID = 34;

        private static final double ENCODER_19T_OFFSET = -0.092041015625;
        private static final double ENCODER_21T_OFFSET = -0.253662109375;

        private static final double MOTOR_TO_TURRET_RATIO = 10.0;
    }

    private static class Reseed {
        private static final Angle POSITION_ERROR_THRESHOLD = Degrees.of(4.0);
        private static final AngularVelocity STATIONARY_VELOCITY_THRESHOLD = RadiansPerSecond.of(Math.toRadians(5.0));
        private static final int REQUIRED_SAMPLES = 10;
        private static final int CRT_MEDIAN_TAPS = 5;
        private static final Angle DEFAULT_ANGLE = Degrees.of(0.0);
    }

    private static class CRT {
        private static final long ENCODER_19T_TEETH = 19L;
        private static final long ENCODER_21T_TEETH = 21L;
        private static final long ENCODER_19T_WRAP = 21L * 10L;
        private static final long ENCODER_21T_WRAP = 19L * 10L;
        private static final long TURRET_GEAR_TEETH = 200L;
        private static final long CRT_MODULUS = 399L;
    }

    private static class Control {
        private static final MagnetSensorConfigs ENCODER_19T_MAGNET_CONFIG = new MagnetSensorConfigs()
                .withMagnetOffset(CANConfiguration.ENCODER_19T_OFFSET)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        private static final MagnetSensorConfigs ENCODER_21T_MAGNET_CONFIG = new MagnetSensorConfigs()
                .withMagnetOffset(CANConfiguration.ENCODER_21T_OFFSET)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        private static final CANcoderConfiguration ENCODER_19T_CONFIG = new CANcoderConfiguration()
                .withMagnetSensor(ENCODER_19T_MAGNET_CONFIG);

        private static final CANcoderConfiguration ENCODER_21T_CONFIG = new CANcoderConfiguration()
                .withMagnetSensor(ENCODER_21T_MAGNET_CONFIG);

        private static final Slot0Configs PIVOT_PID_CONSTANTS = new Slot0Configs()
                .withKS(22)
                .withKP(50)
                .withKI(3)
                .withKD(5)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        private static final MotionMagicConfigs PIVOT_MOTION_MAGIC_CONFIG = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(1);

        private static final CurrentLimitsConfigs PIVOT_MOTOR_CURRENT_LIMITS_CONFIGS = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Amps.of(80))
                .withSupplyCurrentLimitEnable(true);

        private static final SoftwareLimitSwitchConfigs PIVOT_SOFTWARE_LIMITS = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Turret.Constants.MAX_ANGLE)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Turret.Constants.MIN_ANGLE)
                .withReverseSoftLimitEnable(true);

        private static final MotorOutputConfigs PIVOT_MOTOR_OUTPUT_CONFIG = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        private static final TalonFXConfiguration PIVOT_MOTOR_CONFIG = new TalonFXConfiguration()
                .withSlot0(PIVOT_PID_CONSTANTS)
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
                .withMotionMagic(PIVOT_MOTION_MAGIC_CONFIG)
                .withMotorOutput(PIVOT_MOTOR_OUTPUT_CONFIG)
                .withSoftwareLimitSwitch(PIVOT_SOFTWARE_LIMITS)
                .withCurrentLimits(PIVOT_MOTOR_CURRENT_LIMITS_CONFIGS)
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(CANConfiguration.MOTOR_TO_TURRET_RATIO));

        private static final double SIGNAL_UPDATE_HZ = 50.0;
    }

    private final TalonFX pivotMotor = new TalonFX(CANConfiguration.PIVOT_MOTOR_ID, RobotConstants.canBus);
    private final CANcoder encoder19T = new CANcoder(CANConfiguration.ENCODER_19T_ID, RobotConstants.canBus);
    private final CANcoder encoder21T = new CANcoder(CANConfiguration.ENCODER_21T_ID, RobotConstants.canBus);

    private final StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
    private final StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
    private final StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
    private final StatusSignal<Current> pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
    private final StatusSignal<Angle> encoder19TAbsolutePosition = encoder19T.getAbsolutePosition();
    private final StatusSignal<Angle> encoder21TAbsolutePosition = encoder21T.getAbsolutePosition();

    private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    private final MedianFilter crtMedianFilter = new MedianFilter(Reseed.CRT_MEDIAN_TAPS);

    private Angle targetAngle = Reseed.DEFAULT_ANGLE;
    private int reseedCounter = 0;

    public TurretIOReal() {
        applyConfiguration("Turret/Encoder19TConfigStatus",
                encoder19T.getConfigurator().apply(Control.ENCODER_19T_CONFIG));
        applyConfiguration("Turret/Encoder21TConfigStatus",
                encoder21T.getConfigurator().apply(Control.ENCODER_21T_CONFIG));
        applyConfiguration("Turret/PivotMotorConfigStatus",
                pivotMotor.getConfigurator().apply(Control.PIVOT_MOTOR_CONFIG));

        BaseStatusSignal.setUpdateFrequencyForAll(
                Control.SIGNAL_UPDATE_HZ,
                pivotPosition,
                pivotVelocity,
                pivotMotorVoltage,
                pivotTorqueCurrent,
                encoder19TAbsolutePosition,
                encoder21TAbsolutePosition);
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, encoder19T, encoder21T);

        BaseStatusSignal.refreshAll(pivotPosition, encoder19TAbsolutePosition, encoder21TAbsolutePosition);

        Angle initialAngle = getSeedAngle();
        targetAngle = initialAngle;
        reseedPosition(initialAngle);
    }

    private void applyConfiguration(String key, StatusCode status) {
        Logger.recordOutput(key, status.getName());
    }

    private Angle getAbsoluteCrtAngle() {
        double raw19TTeeth = encoder19TAbsolutePosition.getValueAsDouble() * CRT.ENCODER_19T_TEETH;
        double raw21TTeeth = encoder21TAbsolutePosition.getValueAsDouble() * CRT.ENCODER_21T_TEETH;

        long wrapped19TTeeth = Math.round(raw19TTeeth) % CRT.ENCODER_19T_TEETH;
        long wrapped21TTeeth = Math.round(raw21TTeeth) % CRT.ENCODER_21T_TEETH;
        long coarseTurretTeeth = (wrapped19TTeeth * CRT.ENCODER_19T_WRAP + wrapped21TTeeth * CRT.ENCODER_21T_WRAP)
                % CRT.CRT_MODULUS;

        double fractional21TTooth = raw21TTeeth - Math.round(raw21TTeeth);
        double turretGearTeeth = coarseTurretTeeth + fractional21TTooth;
        double turretAngleRadians = MathUtil.inputModulus(
                turretGearTeeth * ((2.0 * Math.PI) / CRT.TURRET_GEAR_TEETH),
                0.0,
                2.0 * Math.PI);

        return Radians.of(turretAngleRadians);
    }

    private Angle seedAngleFromCrt(Angle crtAngle) {
        return Turret.normalizeAngle(Radians.of(crtAngle.in(Radians) + Math.PI));
    }

    private Angle getSeedAngle() {
        return seedAngleFromCrt(getAbsoluteCrtAngle());
    }

    private void reseedPosition(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    private static boolean isWithinLimits(Angle angle) {
        return angle.gte(Turret.Constants.MIN_ANGLE) && angle.lte(Turret.Constants.MAX_ANGLE);
    }

    private Optional<Angle> getCrtFilteredAngle() {
        double filteredRadians = crtMedianFilter.calculate(getSeedAngle().in(Radians));
        if (!Double.isFinite(filteredRadians)) {
            return Optional.empty();
        }

        return Optional.of(Radians.of(filteredRadians));
    }

    private boolean shouldReseed(Optional<Angle> crtAngle, Angle position, AngularVelocity velocity) {
        if (crtAngle.isEmpty() || !isWithinLimits(crtAngle.get())) {
            return false;
        }

        Angle positionError = crtAngle.get().minus(position);
        return Math.abs(velocity.in(RadiansPerSecond)) <= Reseed.STATIONARY_VELOCITY_THRESHOLD.in(RadiansPerSecond)
                && Math.abs(positionError.in(Radians)) >= Reseed.POSITION_ERROR_THRESHOLD.in(Radians);
    }

    private void updateReseedState(TurretIOInputs inputs, Optional<Angle> crtAngle, Angle position,
            AngularVelocity velocity) {
        reseedCounter = shouldReseed(crtAngle, position, velocity) ? reseedCounter + 1 : 0;

        if (reseedCounter >= Reseed.REQUIRED_SAMPLES && crtAngle.isPresent()) {
            reseedPosition(crtAngle.get());
            reseedCounter = 0;
        }

        inputs.motorPositionErrorCounter = reseedCounter;
    }

    @Override
    public void setTarget(Angle angle) {
        targetAngle = angle;
        pivotMotor.setControl(positionControl.withPosition(angle));
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        StatusCode signalStatus = BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotMotorVoltage,
                pivotTorqueCurrent,
                encoder19TAbsolutePosition,
                encoder21TAbsolutePosition);

        Angle position = Turret.normalizeAngle(pivotPosition.getValue());
        AngularVelocity velocity = pivotVelocity.getValue();
        Optional<Angle> crtAngle = getCrtFilteredAngle();

        inputs.goalPosition = targetAngle;
        inputs.currentTurretPosition = position;
        inputs.crtTurretPosition = crtAngle.orElse(Degrees.of(0.0));
        inputs.hasValidCRT = signalStatus.isOK() && crtAngle.isPresent();
        inputs.turretVelocity = velocity;
        inputs.appliedVoltage = pivotMotorVoltage.getValue();
        inputs.appliedCurrent = pivotTorqueCurrent.getValue();

        updateReseedState(inputs, crtAngle, position, velocity);
    }

    @Override
    public void setCurrent(double amps) {
        pivotMotor.setControl(currentControl.withOutput(amps));
    }
}
