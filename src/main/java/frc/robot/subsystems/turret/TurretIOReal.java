package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.OptionalDouble;
import java.util.OptionalLong;

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
        
        static final double ENCODER_19T_OFFSET = -0.7822265625;
        static final double ENCODER_21T_OFFSET = -0.46826171875;
        static final double MOTOR_TO_TURRET_RATIO = 10.0;

        // CRT
        static final long ENCODER_19T_TEETH = 19L;
        static final long ENCODER_21T_TEETH = 21L;
        static final long ENCODER_19T_WRAP = 21L * 10L;
        static final long ENCODER_21T_WRAP = 19L * 10L;
        static final long TURRET_GEAR_TEETH = 200L;
        static final long CRT_MODULUS = 399L;
        static final double CRT_TO_MOTOR_OFFSET_RAD = Math.PI;
        static final double CRT_TO_MOTOR_OFFSET_TEETH = TURRET_GEAR_TEETH / 2.0;

        // Reseeding
        static final AngularVelocity RESEED_VELOCITY_THRESHOLD = RadiansPerSecond.of(Math.toRadians(5.0));
        static final int RESEED_REQUIRED_SAMPLES = 10;
        static final int CRT_MEDIAN_TAPS = 5;

        // Pre-converted travel limits for hot-path comparisons
        static final double MIN_ANGLE_RAD = Turret.Constants.MIN_ANGLE.in(Radians);
        static final double MAX_ANGLE_RAD = Turret.Constants.MAX_ANGLE.in(Radians);
        static final double MIN_MOTOR_TEETH = MIN_ANGLE_RAD * TURRET_GEAR_TEETH / (2.0 * Math.PI);
        static final double MAX_MOTOR_TEETH = MAX_ANGLE_RAD * TURRET_GEAR_TEETH / (2.0 * Math.PI);
        static final double MIN_CRT_TEETH = MIN_MOTOR_TEETH - CRT_TO_MOTOR_OFFSET_TEETH;
        static final double MAX_CRT_TEETH = MAX_MOTOR_TEETH - CRT_TO_MOTOR_OFFSET_TEETH;

        // Signals
        static final double SIGNAL_UPDATE_HZ = 50.0;

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
    private final StatusSignal<Angle> encoder21TPosition = encoder21T.getAbsolutePosition();
    private final StatusSignalCollection turretSignals = new StatusSignalCollection();

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
                pivotPosition, pivotVelocity, pivotMotorVoltage, pivotTorqueCurrent,
                encoder19TPosition, encoder21TPosition);
        turretSignals.setUpdateFrequencyForAll(Constants.SIGNAL_UPDATE_HZ);
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor, encoder19T, encoder21T);

        StatusCode initStatus = turretSignals.refreshAll();

        OptionalDouble initialSeed = getSeedAngleRad();
        if (initStatus.isOK() && initialSeed.isPresent() && isWithinLimits(initialSeed.getAsDouble())) {
            reseedPosition(Radians.of(initialSeed.getAsDouble()));
        } else {
            reseedPosition(Degrees.of(180));
        }
    }

    private void applyConfiguration(String key, StatusCode status) {
        Logger.recordOutput(key, status.getName());
    }

    /**
     * Iterative error-minimizing CRT. Tries all 4 floor/ceil rounding
     * combinations for the two encoders and picks the candidate turret tooth
     * whose predicted encoder readings best match the raw observations.
     */
    static OptionalDouble decodeAbsoluteCrtAngleRad(double encoder19TPositionRotations,
            double encoder21TPositionRotations) {
        OptionalDouble turretTeeth = decodeAbsoluteCrtTeeth(encoder19TPositionRotations, encoder21TPositionRotations);
        if (turretTeeth.isEmpty()) return OptionalDouble.empty();

        return OptionalDouble.of(
                MathUtil.inputModulus(turretTeeth.getAsDouble() * (2.0 * Math.PI / Constants.TURRET_GEAR_TEETH),
                        0.0, 2.0 * Math.PI));
    }

    static OptionalDouble decodeAbsoluteCrtTeeth(double encoder19TPositionRotations,
            double encoder21TPositionRotations) {
        double raw19T = encoder19TPositionRotations * Constants.ENCODER_19T_TEETH;
        double raw21T = encoder21TPositionRotations * Constants.ENCODER_21T_TEETH;

        long floor19T = (long) Math.floor(raw19T);
        long floor21T = (long) Math.floor(raw21T);

        double bestError = Double.MAX_VALUE;
        long bestTooth = 0;
        boolean foundCandidate = false;

        for (int i = 0; i < 2; i++) {
            long a1 = Math.floorMod(floor19T + i, Constants.ENCODER_19T_TEETH);
            for (int j = 0; j < 2; j++) {
                long a2 = Math.floorMod(floor21T + j, Constants.ENCODER_21T_TEETH);

                long residue = Math.floorMod(
                        a1 * Constants.ENCODER_19T_WRAP + a2 * Constants.ENCODER_21T_WRAP,
                        Constants.CRT_MODULUS);
                OptionalLong candidateOpt = candidateToothInSoftRange(residue);
                if (candidateOpt.isEmpty()) continue;

                long candidateTooth = candidateOpt.getAsLong();

                double err19T = toothError(raw19T, Math.floorMod(candidateTooth, Constants.ENCODER_19T_TEETH),
                        Constants.ENCODER_19T_TEETH);
                double err21T = toothError(raw21T, Math.floorMod(candidateTooth, Constants.ENCODER_21T_TEETH),
                        Constants.ENCODER_21T_TEETH);
                double totalError = err19T * err19T + err21T * err21T;

                if (totalError < bestError) {
                    bestError = totalError;
                    bestTooth = candidateTooth;
                    foundCandidate = true;
                }
            }
        }

        if (!foundCandidate) return OptionalDouble.empty();

        // Sub-tooth interpolation from the 21T encoder
        double predicted21T = Math.floorMod(bestTooth, Constants.ENCODER_21T_TEETH);
        double fractional = centeredModulus(raw21T - predicted21T, 0.5);
        double turretTeeth = bestTooth + fractional;
        if (turretTeeth < Constants.MIN_CRT_TEETH || turretTeeth > Constants.MAX_CRT_TEETH) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(turretTeeth);
    }

    private OptionalDouble getAbsoluteCrtAngleRad() {
        return decodeAbsoluteCrtAngleRad(encoder19TPosition.getValueAsDouble(), encoder21TPosition.getValueAsDouble());
    }

    private static double centeredModulus(double value, double halfRange) {
        double modulus = halfRange * 2.0;
        double wrapped = value % modulus;

        if (wrapped > halfRange) wrapped -= modulus;
        if (wrapped < -halfRange) wrapped += modulus;

        return wrapped;
    }

    private static OptionalLong candidateToothInSoftRange(long residue) {
        long candidateTooth = residue;
        double searchMax = Constants.MAX_CRT_TEETH + 0.5;
        double searchMin = Constants.MIN_CRT_TEETH - 0.5;

        if (candidateTooth > searchMax) candidateTooth -= Constants.CRT_MODULUS;
        if (candidateTooth < searchMin || candidateTooth > searchMax) return OptionalLong.empty();

        return OptionalLong.of(candidateTooth);
    }

    private static double toothError(double raw, double predicted, long teeth) {
        return Math.abs(centeredModulus(raw - predicted, teeth / 2.0));
    }

    /** CRT angle converted to motor frame (CRT zero is offset by π from motor zero). */
    static OptionalDouble decodeSeedAngleRad(double encoder19TPositionRotations, double encoder21TPositionRotations) {
        OptionalDouble crtTeeth = decodeAbsoluteCrtTeeth(encoder19TPositionRotations, encoder21TPositionRotations);
        if (crtTeeth.isEmpty()) return OptionalDouble.empty();

        double motorTeeth = crtTeeth.getAsDouble() + Constants.CRT_TO_MOTOR_OFFSET_TEETH;
        return OptionalDouble.of(motorTeeth * (2.0 * Math.PI / Constants.TURRET_GEAR_TEETH));
    }

    private OptionalDouble getSeedAngleRad() {
        return decodeSeedAngleRad(encoder19TPosition.getValueAsDouble(), encoder21TPosition.getValueAsDouble());
    }

    private OptionalDouble getCrtFilteredRad() {
        OptionalDouble seedOpt = getSeedAngleRad();
        if (seedOpt.isEmpty()) return OptionalDouble.empty();
        double v = crtMedianFilter.calculate(seedOpt.getAsDouble());
        return Double.isFinite(v) ? OptionalDouble.of(v) : OptionalDouble.empty();
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
        return crtRad.isPresent()
                && isWithinLimits(crtRad.getAsDouble())
                && Math.abs(velocity.in(RadiansPerSecond)) <= Constants.RESEED_VELOCITY_THRESHOLD.in(RadiansPerSecond);
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
        StatusCode signalStatus = turretSignals.refreshAll();

        double positionRad = MathUtil.inputModulus(pivotPosition.getValue().in(Radians), 0.0, 2.0 * Math.PI);
        AngularVelocity velocity = pivotVelocity.getValue();
        OptionalDouble crtRad = getCrtFilteredRad();

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
