package frc.robot.subsystems.hopperlanes;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.SysIDMechanism;

public class HopperLanes extends SubsystemBase implements SysIDMechanism {
    private static final AngularVelocity indexerVelocityTolerance = RPM.of(20);

    private static final AngularVelocity indexerVelocity = RPM.of(480);

    private final HopperLanesIO io;
    private final HopperLanesInputsAutoLogged inputs = new HopperLanesInputsAutoLogged();

    private SysIdRoutine sysId;

    private final LoggedNetworkNumber tuneIndexerVelocity = new LoggedNetworkNumber("Tuning/IndexerVelocityRPM",
            indexerVelocity.in(RPM));

    @AutoLogOutput
    public final Trigger isStuck = new Trigger(
            () -> Math.abs(inputs.velocityRPM) < 10 && inputs.appliedTorqueCurrent > 5);

    public HopperLanes(HopperLanesIO io) {
        this.io = io;

        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null,
                        (state) -> Logger.recordOutput("HopperLanes/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        io::runVolts, null, this));
    }

    public Command feed() {
        return runEnd(
                () -> io.setVelocity(indexerVelocity),
                io::stopMotor);
    }

    // public Command feed() {
    // return Commands.either(reverse(), runEnd(
    // () -> io.setVelocity(RPM.of(tuneIndexerVelocity.get())),
    // io::stopMotor), isStuck);
    // }

    public final Trigger atSpeed = new Trigger(
            () -> RPM.of(inputs.velocityRPM).isNear(indexerVelocity, indexerVelocityTolerance));

    public Command feedTunable() {
        return runEnd(
                () -> io.setVelocity(RPM.of(tuneIndexerVelocity.get())),
                io::stopMotor);
    }

    public Command reverse() {
        return runEnd(() -> io.runVolts(Volts.of(-6)), io::stopMotor);
    }

    public AngularVelocity getVelocity() {
        return RPM.of(inputs.velocityRPM);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @Override
    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        return List.of(SysIDMechanism.named("Hopper Lanes", this::sysIdDynamic, this::sysIdQuasistatic));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("HopperLanes", inputs);
    }
}
