package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class Turret extends SubsystemBase implements SysIDMechanism {
    public static class Constants {
        private static final Angle CONTROL_TOLERANCE = Degrees.of(3);
        private static final AngularVelocity READY_VELOCITY_TOLERANCE = RadiansPerSecond.of(Math.toRadians(8.0));

        // The reachable travel range wraps around +X, so robot-relative zero lies in
        // the deadzone.
        public static final Angle MIN_ANGLE = Degrees.of(40);
        public static final Angle MAX_ANGLE = Degrees.of(277.87);
    }

    private final TurretIO pivotMotor;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private Angle currentTarget = Rotations.of(0.5);

    @AutoLogOutput
    public final Trigger isAtTarget;

    @AutoLogOutput
    public final Trigger isAtLimit;

    private static final LoggedNetworkNumber tuneAngle = new LoggedNetworkNumber("Tuning/TuneTurretAngle", 180);

    public Turret(TurretIO pivotMotor) {
        this.pivotMotor = pivotMotor;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(2).per(Second), Volts.of(20), Seconds.of(10),
                        (state) -> Logger.recordOutput("Turret/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> pivotMotor.setCurrent(voltage.in(Volts)), null, this));

        isAtTarget = new Trigger(() -> checkAtAngle(currentTarget));
        isAtLimit = new Trigger(() -> currentTarget.lte(Constants.MIN_ANGLE) || currentTarget.gte(Constants.MAX_ANGLE));

        setDefaultCommand(setAngle(Degrees.of(180)));
    }

    public static Angle normalizeAngle(Angle angle) {
        return Radians.of(MathUtil.inputModulus(angle.in(Radians), 0.0, 2.0 * Math.PI));
    }

    public static boolean isWithinTravelRange(Angle angle) {
        double rad = normalizeAngle(angle).in(Radians);
        return rad >= Constants.MIN_ANGLE.in(Radians) && rad <= Constants.MAX_ANGLE.in(Radians);
    }

    private static Angle getClosestBoundary(Angle normalizedAngle) {
        double minDist = Math.abs(MathUtil.inputModulus(
                normalizedAngle.in(Radians) - Constants.MIN_ANGLE.in(Radians), -Math.PI, Math.PI));
        double maxDist = Math.abs(MathUtil.inputModulus(
                normalizedAngle.in(Radians) - Constants.MAX_ANGLE.in(Radians), -Math.PI, Math.PI));
        return minDist <= maxDist ? Constants.MIN_ANGLE : Constants.MAX_ANGLE;
    }

    public static Angle clampAngle(Angle angle) {
        Angle normalized = normalizeAngle(angle);
        return isWithinTravelRange(normalized) ? normalized : getClosestBoundary(normalized);
    }

    public static Angle resolveTargetAngle(Angle currentAngle, Angle requestedAngle) {
        Angle normalizedCurrent = normalizeAngle(currentAngle);
        Angle normalizedRequested = normalizeAngle(requestedAngle);

        if (isWithinTravelRange(normalizedRequested)) {
            return normalizedRequested;
        }

        double shortestError = MathUtil.inputModulus(
                normalizedRequested.in(Radians) - normalizedCurrent.in(Radians), -Math.PI, Math.PI);

        if (shortestError > 0.0)
            return Constants.MAX_ANGLE;
        if (shortestError < 0.0)
            return Constants.MIN_ANGLE;
        return getClosestBoundary(normalizedRequested);
    }

    private Angle getTurretPosition() {
        return normalizeAngle(inputs.positionRadians);
    }

    private boolean checkAtAngle(Angle target) {
        Angle position = getTurretPosition();
        return position.isNear(currentTarget, Constants.CONTROL_TOLERANCE);
                // && Math.abs(inputs.velocityRadPerSec.in(RadiansPerSecond)) <= Constants.READY_VELOCITY_TOLERANCE
                //         .in(RadiansPerSecond);
    }

    private void commandTarget(Angle target) {
        Angle position = getTurretPosition();
        Angle resolvedTarget = resolveTargetAngle(position, target);
        currentTarget = resolvedTarget;

        if (!isAtTarget.getAsBoolean()) {
            pivotMotor.setTarget(resolvedTarget);
        } else {
            stop();
        }
    }

    private void stop() {
        pivotMotor.setVoltage(0.0);
    }

    private Command sysIdSettle() {
        return run(this::stop).withTimeout(1.0);
    }

    public Trigger isAtAngle(Angle target) {
        return new Trigger(() -> checkAtAngle(target));
    }

    public Command setAngle(Angle angle) {
        return runEnd(() -> commandTarget(angle), this::stop);
    }

    public Command followAngle(Supplier<Angle> angle) {
        return runEnd(() -> {
            commandTarget(angle.get());
        }, this::stop);
    }

    public Command aimTunable() {
        return followAngle(() -> Degrees.of(tuneAngle.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdSettle().andThen(sysId.quasistatic(direction).finallyDo(this::stop));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdSettle().andThen(sysId.dynamic(direction).finallyDo(this::stop));
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Turret", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        pivotMotor.updateInputs(inputs);
        inputs.goalPositionRadians.mut_replace(currentTarget);
        Logger.processInputs("Turret", inputs);
    }
}
