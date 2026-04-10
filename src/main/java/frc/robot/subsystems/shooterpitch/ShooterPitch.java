package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class ShooterPitch extends SubsystemBase implements SysIDMechanism {
    public static class Constants {
        public static final Angle MAX_ANGLE = Degree.of(34.5);
        public static final Angle MIN_ANGLE = Degree.of(5.361328);

        static final Angle ANGLE_TOLERANCE = Degrees.of(1);
        static final double READY_VELOCITY_TOLERANCE_RAD_PER_SEC = Math.toRadians(5.0);

        static final Angle ZERO_ANGLE = Degrees.of(0);
        static final double DEFAULT_HOMING_VOLTAGE = -3;
        static final double DEFAULT_HOMING_CURRENT_SPIKE_AMPS = 30.0;

        // Current-based SysID: values passed to WPILib as "volts" but applied as amps.
        // Very limited travel (~29°) — short timeout; soft limits catch the ends.
        static final double SYSID_RAMP_AMPS_PER_SEC = 3.0;
        static final double SYSID_STEP_AMPS = 15.0;
        static final double SYSID_TIMEOUT_SECS = 5.0;
    }

    private final ShooterPitchIO pitchMotor;
    private final ShooterPitchInputsAutoLogged inputs = new ShooterPitchInputsAutoLogged();
    private final SysIdRoutine sysId;

    private final LoggedNetworkNumber tuneAngle = new LoggedNetworkNumber(
        "Tuning/ShooterPitch", Constants.MIN_ANGLE.in(Degrees));
    private final LoggedNetworkNumber homingVoltage = new LoggedNetworkNumber(
        "Tuning/ShooterPitchHomingVoltage", Constants.DEFAULT_HOMING_VOLTAGE);
    private final LoggedNetworkNumber homingCurrentSpikeAmps = new LoggedNetworkNumber(
        "Tuning/ShooterPitchHomingCurrentSpikeAmps", Constants.DEFAULT_HOMING_CURRENT_SPIKE_AMPS);

    public ShooterPitch(ShooterPitchIO pitchMotor) {
        this.pitchMotor = pitchMotor;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                // Lie to WPILib: these are A/s and A respectively, not V/s and V.
                Volts.of(Constants.SYSID_RAMP_AMPS_PER_SEC).per(Second),
                Volts.of(Constants.SYSID_STEP_AMPS),
                Seconds.of(Constants.SYSID_TIMEOUT_SECS),
                (state) -> Logger.recordOutput("ShooterPitch/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                // Reinterpret the "voltage" magnitude as amps.
                (fakeVolts) -> pitchMotor.runCurrent(Amps.of(fakeVolts.in(Volts))),
                null, this));

        setDefaultCommand(Commands.either(homeAndReseed(), holdMinAngle(), pitchMotor::supportsHomingReseed));
    }

    public Angle getPitchPosition() {
        return inputs.position;
    }

    public Trigger isAtAngle(Angle target) {
        return new Trigger(() ->
            inputs.position.isNear(target, Constants.ANGLE_TOLERANCE)
            && Math.abs(inputs.velocity.in(RadiansPerSecond)) <= Constants.READY_VELOCITY_TOLERANCE_RAD_PER_SEC);
    }

    private boolean hasHomingCurrentSpike() {
        return Math.abs(inputs.appliedCurrent.in(Amps)) >= homingCurrentSpikeAmps.get();
    }

    private Command holdMinAngle() {
        return run(() -> pitchMotor.setTarget(Constants.MIN_ANGLE));
    }

    public Command homeAndReseed() {
        return Commands.sequence(
            runEnd(
                () -> pitchMotor.setHomingVoltage(Volts.of(homingVoltage.get())),
                pitchMotor::stop)
                .until(this::hasHomingCurrentSpike),
            runOnce(() -> pitchMotor.reseedPosition(Constants.MIN_ANGLE)),
            holdMinAngle());
    }

    public Command setAngle(Angle pitchAngle) {
        return runEnd(
            () -> pitchMotor.setTarget(pitchAngle),
            () -> pitchMotor.setTarget(Constants.ZERO_ANGLE));
    }

    public Command tuneAngle() {
        return followAngle(() -> Degrees.of(tuneAngle.get()));
    }

    public Command followAngle(Supplier<Angle> pitchAngle) {
        return runEnd(
            () -> pitchMotor.setTarget(pitchAngle.get()),
            () -> pitchMotor.setTarget(Constants.ZERO_ANGLE));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        // Settle for 1s before running to ensure mechanism is stationary.
        return runOnce(pitchMotor::stop)
            .withTimeout(1.0)
            .andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return runOnce(pitchMotor::stop)
            .withTimeout(1.0)
            .andThen(sysId.dynamic(direction));
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Shooter Pitch", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        pitchMotor.updateInputs(inputs);
        Logger.processInputs("ShooterPitch", inputs);
    }
}
