package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.List;
import java.util.Objects;
import java.util.function.Function;

public interface SysIDMechanism {
    List<NamedMechanism> sysIdMechanisms();

    static NamedMechanism named(
            String name,
            Function<SysIdRoutine.Direction, Command> dynamic,
            Function<SysIdRoutine.Direction, Command> quasistatic) {
        return new NamedMechanism(name, dynamic, quasistatic);
    }

    record NamedMechanism(
            String name,
            Function<SysIdRoutine.Direction, Command> dynamic,
            Function<SysIdRoutine.Direction, Command> quasistatic) {
        public NamedMechanism {
            Objects.requireNonNull(name, "name");
            Objects.requireNonNull(dynamic, "dynamic");
            Objects.requireNonNull(quasistatic, "quasistatic");
        }
    }
}
