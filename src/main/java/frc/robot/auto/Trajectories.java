package frc.robot.auto;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.supersystems.shooter.Shooter;
import frc.robot.util.SubsystemRegistry;

public final class Trajectories {
    private Trajectories() {}

    public static void addAutos(AutoChooser chooser, Autos autos, SubsystemRegistry subsystems) {
        chooser.addRoutine("TUNE_MOI", () -> tuneMoi(autos));
        chooser.addCmd("Trench2xOutpost", () -> trench2xOutpost(
            autos,
            subsystems.get(IntakePivot.class).get(),
            subsystems.get(IntakeRollers.class).get(),
            subsystems.get(Shooter.class).get()
        ));
    }

    private static Command trench2xOutpost(Autos autos, IntakePivot intakePivot, IntakeRollers intakeRollers, Shooter shooter) {
        AutoRoutine routine = autos.routine("Trench2xOutpost");
        AutoTrajectory[] paths = autos.split(routine, ChoreoTraj.Trench2xOutpost);

        return Commands.sequence(
            paths[0].resetOdometry(),
            intakePivot.deploy(), // deploy intake
            paths[0].cmd(), // drive from start to pre-sweep
            Commands.deadline( // first sweep + rollers active
                paths[1].cmd(),
                intakeRollers.intake()
            ),
            paths[2].cmd(), // drive to shoot
            shooter.shoot() // shoot
                .withTimeout(Seconds.of(4)),
            paths[3].cmd(), // drive to pre-sweep
            Commands.deadline( // second sweep + rollers active
                paths[4].cmd(),
                intakeRollers.intake()
            ),
            paths[5].cmd(), // drive to shoot
            shooter.shoot()
                .withTimeout(Seconds.of(4)),
            autos.stopCommand()
        );
    }

    private static AutoRoutine tuneMoi(Autos autos) {
        AutoRoutine routine = autos.routine("TUNE_MOI");
        AutoTrajectory path = autos.trajectory(routine, ChoreoTraj.CalibrateMOI);

        autos.start(routine, path);
        autos.finish(path, autos.stopCommand());

        return routine;
    }
}
