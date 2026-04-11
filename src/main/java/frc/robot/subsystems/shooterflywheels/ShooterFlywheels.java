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
                Volts.of(3.0).per(Second),
                Volts.of(30.0),
                Seconds.of(10.0),
                (state) -> Logger.recordOutput("ShooterFlywheels/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
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
