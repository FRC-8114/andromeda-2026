package frc.robot.subsystems.turretfeeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class TurretFeeder extends SubsystemBase implements SysIDMechanism {
    private static final AngularVelocity velocityTolerance = RPM.of(30);
    private static final AngularVelocity turretLoaderVelocity = RPM.of(2000);

    private final TurretFeederIO io;
    private final TurretFeederInputsAutoLogged inputs = new TurretFeederInputsAutoLogged();

    private SysIdRoutine sysId;

    private final LoggedNetworkNumber tuneTurretLoaderVelocity = new LoggedNetworkNumber(
            "Tuning/TurretLoaderVelocityRPM", turretLoaderVelocity.in(RPM));

    public TurretFeeder(TurretFeederIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(3).per(Second), Volts.of(50), Seconds.of(20),
                        (state) -> Logger.recordOutput("TurretFeeder/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> io.runTorqueCurrent(Amps.of(voltage.in(Volts))), null, this));
    }

    public final Trigger atSpeed = new Trigger(
        () -> RPM.of(inputs.beltVelocityRPM).isNear(turretLoaderVelocity, velocityTolerance));

    public Command feed() {
        return runEnd(
            () -> io.setVelocity(turretLoaderVelocity),
            () -> io.stopMotor()
        );
    }

    public Command feedTunable() {
        return runEnd(
            () -> io.setVelocity(RPM.of(tuneTurretLoaderVelocity.get())),
            () -> io.stopMotor()
        );
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.beltVelocityRPM);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Turret Feeder", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("TurretFeeder", inputs);
    }
}
