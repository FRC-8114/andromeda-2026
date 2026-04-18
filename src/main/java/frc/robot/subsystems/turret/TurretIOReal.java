package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

public class TurretIOReal implements TurretIO {
    private static class Constants {
        // Hardware
        static final int PIVOT_MOTOR_ID = 32;
        static final int ENCODER_19T_ID = 33;
        static final int ENCODER_21T_ID = 34;

        static final double ENCODER_19T_OFFSET = -0.784912109375;
        static final double ENCODER_21T_OFFSET = -0.473876953125;
        static final double MOTOR_TO_TURRET_RATIO = 10.0;

        // CRT
        static final long ENCODER_19T_TEETH = 19L;
        static final long ENCODER_21T_TEETH = 21L;
        static final long TURRET_GEAR_TEETH = 200L;
        static final double CRT_COHERENCE_TOLERANCE_TEETH = 5;
        static final Angle CRT_ZERO_OFFSET = Rotations.of(0.5);

        // Reseeding
        static final AngularVelocity RESEED_VELOCITY_THRESHOLD = RadiansPerSecond.of(Math.toRadians(5.0));
        static final int RESEED_REQUIRED_SAMPLES = 10;
        static final int CRT_MEDIAN_TAPS = 5;

        // Pre-converted travel limits for hot-path comparisons
        static final double MIN_ANGLE_RAD = Turret.Constants.MIN_ANGLE.in(Radians);
        static final double MAX_ANGLE_RAD = Turret.Constants.MAX_ANGLE.in(Radians);

        // Signals
        static final double POSITIONAL_SIGNAL_UPDATE_HZ = 50.0;

        // Configs
        static final MagnetSensorConfigs ENCODER_19T_MAGNET_CONFIG = new MagnetSensorConfigs()
                .withMagnetOffset(ENCODER_19T_OFFSET)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        static final MagnetSensorConfigs ENCODER_21T_MAGNET_CONFIG = new MagnetSensorConfigs()
                .withMagnetOffset(ENCODER_21T_OFFSET)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withAbsoluteSensorDiscontinuityPoint(1);

        static final CANcoderConfiguration ENCODER_19T_CONFIG = new CANcoderConfiguration()
                .withMagnetSensor(ENCODER_19T_MAGNET_CONFIG);

        static final CANcoderConfiguration ENCODER_21T_CONFIG = new CANcoderConfiguration()
                .withMagnetSensor(ENCODER_21T_MAGNET_CONFIG);

        static final Slot0Configs PIVOT_SLOT0 = new Slot0Configs()
                .withKS(2)
                .withKP(8)
                .withKD(0.9)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        static final MotionMagicConfigs PIVOT_MOTION_MAGIC = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(1);

        static final CurrentLimitsConfigs PIVOT_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(60))
                .withSupplyCurrentLimitEnable(true);

        static final SoftwareLimitSwitchConfigs PIVOT_SOFT_LIMITS = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(Turret.Constants.MAX_ANGLE)
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Turret.Constants.MIN_ANGLE)
                .withReverseSoftLimitEnable(true);

        static final MotorOutputConfigs PIVOT_OUTPUT = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        static final TalonFXConfiguration PIVOT_CONFIG = new TalonFXConfiguration()
                .withSlot0(PIVOT_SLOT0)
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(false))
                .withMotionMagic(PIVOT_MOTION_MAGIC)
                .withMotorOutput(PIVOT_OUTPUT)
                .withSoftwareLimitSwitch(PIVOT_SOFT_LIMITS)
                .withCurrentLimits(PIVOT_CURRENT_LIMITS)
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(MOTOR_TO_TURRET_RATIO));
    }

    private final TalonFX pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID, RobotConstants.canBus);
    private final CANcoder encoder19T = new CANcoder(Constants.ENCODER_19T_ID, RobotConstants.canBus);
    private final CANcoder encoder21T = new CANcoder(Constants.ENCODER_21T_ID, RobotConstants.canBus);

    private final StatusSignal<Angle> pivotPosition = pivotMotor.getPosition();
    private final StatusSignal<AngularVelocity> pivotVelocity = pivotMotor.getVelocity();
    private final StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
    private final StatusSignal<Current> pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
    private final StatusSignal<Angle> encoder19TPosition = encoder19T.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> encoder19TVelocity = encoder19T.getVelocity();
    private final StatusSignal<Angle> encoder21TPosition = encoder21T.getAbsolutePosition();
    private final StatusSignal<AngularVelocity> encoder21TVelocity = encoder21T.getVelocity();
    private final StatusSignalCollection turretSignals = new StatusSignalCollection();
    private final StatusSignalCollection positionalTurretSignals = new StatusSignalCollection();

    private final PositionVoltage positionControl = new PositionVoltage(Math.PI);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    private final MedianFilter crtMedianFilter = new MedianFilter(Constants.CRT_MEDIAN_TAPS);

    private int reseedCounter = 0;

    public TurretIOReal() {
        applyConfiguration("Turret/Encoder19TConfigStatus",
                encoder19T.getConfigurator().apply(Constants.ENCODER_19T_CONFIG));
        applyConfiguration("Turret/Encoder21TConfigStatus",
                encoder21T.getConfigurator().apply(Constants.ENCODER_21T_CONFIG));
        applyConfiguration("Turret/PivotMotorConfigStatus",
                pivotMotor.getConfigurator().apply(Constants.PIVOT_CONFIG));

        turretSignals.addSignals(
                pivotMotorVoltage, pivotTorqueCurrent);
        turretSignals.setUpdateFrequencyForAll(50);

        positionalTurretSignals.addSignals(pivotPosition, pivotVelocity, encoder19TPosition, encoder21TPosition,
                encoder19TVelocity, encoder21TVelocity);
        positionalTurretSignals.setUpdateFrequencyForAll(Constants.POSITIONAL_SIGNAL_UPDATE_HZ);

        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, encoder19T, encoder21T);

        turretSignals.refreshAll();
        positionalTurretSignals.refreshAll();

        reseedPosition(Degrees.of(180));
        // reseedPosition(Radians.of(getCRTAngleRadians().orElse(180)));
    }

    private void applyConfiguration(String key, StatusCode status) {
        Logger.recordOutput(key, status.getName());
    }

    private final double RATIO_19 = (double) Constants.TURRET_GEAR_TEETH / Constants.ENCODER_19T_TEETH;
    private final double RATIO_21 = (double) Constants.TURRET_GEAR_TEETH / Constants.ENCODER_21T_TEETH;

    private OptionalDouble getAbsoluteCrtAngleRadians() {
    double abs21 = encoder21TPosition.getValueAsDouble();  // pivot
    double abs19 = encoder19TPosition.getValueAsDouble();  // predictor

    double minMechanismRotations = Turret.Constants.MIN_ANGLE.plus(Constants.CRT_ZERO_OFFSET).in(Rotations);
    double maxMechanismRotations = Turret.Constants.MIN_ANGLE.plus(Constants.CRT_ZERO_OFFSET).in(Rotations);

    double matchTolerance = Constants.CRT_COHERENCE_TOLERANCE_TEETH / Constants.ENCODER_19T_TEETH;

    if (
        Math.abs(RATIO_21) < 1e-12
        || !Double.isFinite(minMechanismRotations)
        || !Double.isFinite(maxMechanismRotations)
        || minMechanismRotations > maxMechanismRotations
        || !Double.isFinite(matchTolerance)
        || matchTolerance < 0.0) {
        Logger.recordOutput("Turret/CRTStatus", "INVALID_CONFIG");
        return OptionalDouble.empty();
    }

    double nMinD = Math.min(RATIO_21 * minMechanismRotations,
                            RATIO_21 * maxMechanismRotations) - abs21;
    double nMaxD = Math.max(RATIO_21 * minMechanismRotations,
                            RATIO_21 * maxMechanismRotations) - abs21;
    int minN = (int) Math.floor(nMinD) - 1;
    int maxN = (int) Math.ceil(nMaxD) + 1;

    double bestErr = Double.MAX_VALUE;
    double secondErr = Double.MAX_VALUE;
    double bestRot = Double.NaN;
    int iterations = 0;

    for (int n = minN; n <= maxN; n++) {
        iterations++;

        double mechRot = (abs21 + n) / RATIO_21;
        if (mechRot < minMechanismRotations - 1e-6
         || mechRot > maxMechanismRotations + 1e-6) {
            continue;
        }

        double predicted19 = MathUtil.inputModulus(RATIO_19 * mechRot, 0.0, 1.0);
        double diff = MathUtil.inputModulus(predicted19 - abs19, -0.5, 0.5);
        double err = Math.abs(diff);

        if (err < bestErr) {
            secondErr = bestErr;
            bestErr = err;
            bestRot = mechRot;
        } else if (err < secondErr) {
            secondErr = err;
        }
    }

    Logger.recordOutput("Turret/CRTIterations", iterations);
    Logger.recordOutput("Turret/CRTBestErrorRot", bestErr);
    Logger.recordOutput("Turret/CRTSecondErrorRot", secondErr);

    if (!Double.isFinite(bestRot) || bestErr > matchTolerance) {
        Logger.recordOutput("Turret/CRTStatus", "NO_SOLUTION");
        return OptionalDouble.empty();
    }
    if (secondErr <= matchTolerance && Math.abs(secondErr - bestErr) < 1e-3) {
        Logger.recordOutput("Turret/CRTStatus", "AMBIGUOUS");
        return OptionalDouble.empty();
    }

    Logger.recordOutput("Turret/CRTStatus", "OK");

    double mechRotShifted = bestRot - Constants.CRT_ZERO_OFFSET.in(Rotation);

    return OptionalDouble.of(mechRotShifted * 2.0 * Math.PI);
}

    // crt minus filtering
    private OptionalDouble getCRTAngleRadians() {
        OptionalDouble rawCRT = getAbsoluteCrtAngleRadians();

        if (rawCRT.isEmpty())
            return OptionalDouble.empty();

        return OptionalDouble.of(MathUtil.inputModulus(
                getAbsoluteCrtAngleRadians().getAsDouble() + Constants.CRT_ZERO_OFFSET.in(Radians), 0.0,
                2.0 * Math.PI));
    }

    private OptionalDouble getFilteredCRT() {
        OptionalDouble crt = getCRTAngleRadians();

        if (crt.isEmpty())
            return OptionalDouble.empty();

        double filtered = crtMedianFilter.calculate(crt.getAsDouble());

        return Double.isFinite(filtered) ? OptionalDouble.of(filtered) : OptionalDouble.empty();
    }

    private static boolean isWithinLimits(double angleRad) {
        return angleRad >= Constants.MIN_ANGLE_RAD && angleRad <= Constants.MAX_ANGLE_RAD;
    }

    private void reseedPosition(Angle angle) {
        pivotMotor.setPosition(angle);
    }

    private static double crtPositionErrorRadians(double crtRad, double positionRad) {
        return MathUtil.inputModulus(crtRad - positionRad, -Math.PI, Math.PI);
    }

    private boolean shouldReseed(OptionalDouble crtRad, AngularVelocity velocity) {
        return false;
        // return crtRad.isPresent()
        // && isWithinLimits(crtRad.getAsDouble())
        // && Math.abs(velocity.in(RadiansPerSecond)) <=
        // Constants.RESEED_VELOCITY_THRESHOLD.in(RadiansPerSecond);
    }

    private void updateReseedState(TurretIOInputs inputs, OptionalDouble crtRad, double positionRad,
            AngularVelocity velocity) {
        reseedCounter = shouldReseed(crtRad, velocity) ? reseedCounter + 1 : 0;

        double errorRadians = crtRad.isPresent() ? crtPositionErrorRadians(crtRad.getAsDouble(), positionRad) : 0.0;
        inputs.crtPositionErrorRadians = errorRadians;
        inputs.reseedSampleCount = reseedCounter;

        if (reseedCounter >= Constants.RESEED_REQUIRED_SAMPLES) {
            double crtAngleRad = crtRad.getAsDouble();
            Logger.recordOutput("Turret/ReseedAngleRad", crtAngleRad);
            Logger.recordOutput("Turret/ReseedErrorRad", errorRadians);
            reseedPosition(Radians.of(crtAngleRad));
            reseedCounter = 0;
        }
    }

    @Override
    public void setTarget(Angle angle) {
        pivotMotor.setControl(positionControl.withPosition(angle));
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turretSignals.refreshAll();

        StatusCode signalStatus = positionalTurretSignals.refreshAll();

        double positionRad = MathUtil.inputModulus(pivotPosition.getValue().in(Radians), 0.0, 2.0 * Math.PI);
        AngularVelocity velocity = pivotVelocity.getValue();
        OptionalDouble crtRad = getFilteredCRT();

        inputs.positionRadians.mut_replace(positionRad, Radians);
        if (crtRad.isPresent()) {
            inputs.crtPositionRadians.mut_replace(crtRad.getAsDouble(), Radians);
        } else {
            inputs.crtPositionRadians.mut_replace(0, Degrees);
        }
        inputs.hasValidCRT = signalStatus.isOK() && crtRad.isPresent() && isWithinLimits(crtRad.getAsDouble());
        inputs.velocityRadPerSec.mut_replace(velocity);
        inputs.voltageVolts.mut_replace(pivotMotorVoltage.getValue());
        inputs.currentAmps.mut_replace(pivotTorqueCurrent.getValue());

        updateReseedState(inputs, crtRad, positionRad, velocity);
    }

    @Override
    public void setCurrent(double amps) {
        pivotMotor.setControl(currentControl.withOutput(amps));
    }
}
