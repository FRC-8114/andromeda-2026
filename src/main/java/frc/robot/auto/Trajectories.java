package frc.robot.auto;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.subsystems.intakerollers.IntakeRollers;
import frc.robot.supersystems.intake.Intake;
import frc.robot.supersystems.shooter.Shooter;
import frc.robot.util.SubsystemRegistry;

public final class Trajectories {
    private Trajectories() {
    }

    public static void addAutos(AutoChooser chooser, Autos autos, SubsystemRegistry subsystems) {
        chooser.addRoutine("TUNE_MOI", () -> tuneMoi(autos));
        chooser.addRoutine("Trench2xOutpost", () -> trench2xOutpost(
                autos,
                subsystems.get(Intake.class).get(),
                subsystems.get(Shooter.class).get()));
        chooser.addRoutine("Trench1xDepot", () -> trench1xDepot(
                autos,
                subsystems.get(Intake.class).get(),
                subsystems.get(Shooter.class).get()));
        chooser.addRoutine("ShootDepot", () -> shootDepot(
                autos,
                subsystems.get(Shooter.class).get(),
                subsystems.get(Intake.class).get(),
                subsystems.get(Climber.class).get()));
    }

    private static AutoRoutine trench2xOutpost(Autos autos, Intake intake,
            Shooter shooter) {
        AutoRoutine routine = autos.routine("Trench2xOutpost");
        AutoTrajectory[] paths = autos.split(routine, ChoreoTraj.Trench2xOutpost);

        routine.active().onTrue(Commands.sequence(
                paths[0].resetOdometry(),
                Commands.deadline( // do first sweep + rollers active
                        paths[0].cmd(),
                        intake.intake()),
                paths[1].cmd(), // drive to shoot
                autos.stopCommand(), // NO MORE DRIFTING PLS
                Commands.parallel(
                        intake.pump(),
                        shooter.shoot())
                        .withTimeout(Seconds.of(8)),
                paths[2].cmd(), // drive to pre-sweep
                Commands.deadline( // second sweep + rollers active
                        paths[3].cmd(),
                        intake.intake()),
                paths[4].cmd(), // drive to shoot
                autos.stopCommand(),
                Commands.parallel(
                        intake.pump(),
                        shooter.shoot())
                        .withTimeout(Seconds.of(4)),
                autos.stopCommand()));

        return routine;
    }

    private static AutoRoutine trench1xDepot(Autos autos, Intake intake, Shooter shooter) {
        AutoRoutine routine = autos.routine("Trench1xDepot");
        AutoTrajectory[] paths = autos.split(routine, ChoreoTraj.Trench1xDepot);

        routine.active().onTrue(Commands.sequence(
                paths[0].resetOdometry(),
                Commands.deadline( // do first sweep + rollers active
                        paths[0].cmd(),
                        intake.intake()),
                paths[1].cmd(), // drive to shoot
                autos.stopCommand(), // NO MORE DRIFTING PLS
                Commands.parallel(
                        intake.pump(),
                        shooter.shoot())
                        .withTimeout(Seconds.of(12))
        ));

        return routine;
    }

    private static AutoRoutine shootDepot(Autos autos, Shooter shooter, Intake intake, Climber climber) {
        AutoRoutine routine = autos.routine("ShootDepot");
        var paths = autos.split(routine, ChoreoTraj.ShootDepot);

        routine.active().onTrue(Commands.sequence(
                paths[0].resetOdometry(),
                Commands.deadline(
                        paths[0].cmd(),
                        intake.intake()),
                autos.stopCommand(),
                Commands.parallel(
                        shooter.shoot(),
                        Commands.sequence(
                                intake.intake()
                                        .withTimeout(Seconds.of(3.5)),
                                intake.pump()))
                        .withTimeout(6),
                Commands.parallel(
                        paths[1].cmd(),
                        climber.deploy()),
                paths[2].cmd(),
                autos.stopCommand(),
                climber.climb()));

        return routine;
    }

    private static AutoRoutine tuneMoi(Autos autos) {
        AutoRoutine routine = autos.routine("TUNE_MOI");
        AutoTrajectory path = autos.trajectory(routine, ChoreoTraj.CalibrateMOI);

        autos.start(routine, path);
        autos.finish(path, autos.stopCommand());

        return routine;
    }
}
