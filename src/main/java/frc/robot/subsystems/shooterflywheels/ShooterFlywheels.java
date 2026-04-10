package frc.robot.subsystems.shooterflywheels;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class ShooterFlywheels extends SubsystemBase implements SysIDMechanism {
    private static class Constants {
        static final double FLYWHEEL_TOLERANCE_RPM = 50.0;

        // Current-based SysID: values passed to WPILib as "volts" but applied as amps.
        // 70A stator limit — 30A step gives meaningful spin-up without saturation.
        static final double SYSID_RAMP_AMPS_PER_SEC = 3.0;
        static final double SYSID_STEP_AMPS = 30.0;
        static final double SYSID_TIMEOUT_SECS = 10.0;
    }

    private final ShooterFlywheelsIO io;
    private final ShooterFlywheelsInputsAutoLogged inputs = new ShooterFlywheelsInputsAutoLogged();
    private final SysIdRoutine sysId;

    private static final LoggedNetworkNumber tuneVelocity =
        new LoggedNetworkNumber("Tuning/TuneShooterVelocityRPM", 2000);

    public ShooterFlywheels(ShooterFlywheelsIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                // Lie to WPILib: these are A/s and A respectively, not V/s and V.
                Volts.of(Constants.SYSID_RAMP_AMPS_PER_SEC).per(Second),
                Volts.of(Constants.SYSID_STEP_AMPS),
                Seconds.of(Constants.SYSID_TIMEOUT_SECS),
                (state) -> Logger.recordOutput("ShooterFlywheels/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                // Reinterpret the "voltage" magnitude as amps.
                (fakeVolts) -> io.runCurrent(Amps.of(fakeVolts.in(Volts))),
                null, this));
    }

    public Trigger atSpeed(AngularVelocity goalVelocity) {
        return new Trigger(() -> inputs.leftVelocity.isNear(goalVelocity, Constants.FLYWHEEL_TOLERANCE_RPM));
    }

    public Command runFlywheelsTunableVelocity() {
        return runEnd(
            () -> io.setFlywheelVelocity(RPM.of(tuneVelocity.get())),
            io::stop);
    }

    public Command runFlywheels(AngularVelocity target) {
        return runEnd(
            () -> io.setFlywheelVelocity(target),
            io::stop);
    }

    public Command runFlywheels(Supplier<AngularVelocity> target) {
        return runEnd(
            () -> io.setFlywheelVelocity(target.get()),
            io::stop);
    }

    public Command stopFlywheels() {
        return runOnce(io::stop);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Shooter Flywheels", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ShooterFlywheels", inputs);
    }
}
