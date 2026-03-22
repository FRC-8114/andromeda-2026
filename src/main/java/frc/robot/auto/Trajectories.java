package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakepivot.IntakePivot;
import frc.robot.supersystems.shooter.Shooter;
import frc.robot.util.SubsystemRegistry;

public final class Trajectories {
    private static final double BASIC_SHOOT_START_DELAY_SECS = 2.0;
    private static final double BASIC_SHOOT_SHOOT_TIME_SECS = 6.0;
    private static final double OUTPOST_SHOOT_TIME_SECS = 5.0;
    private static final double OUTPOST_BALL_DROP_WAIT_SECS = 4.0;
    private static final double DEPOT_SHOOT_TIME_SECS = 5.0;
    private static final double LAMEST_AUTO_TOTAL_TIME_SECS = 7.0;

    private Trajectories() {
    }

    public static void addAutos(AutoChooser chooser, Autos autos) {
        chooser.addRoutine("basicShoot", () -> basicShoot(autos));
        chooser.addCmd("driveShoot",
                () -> unavailableAuto("driveShoot", "No driveShoot trajectory exists in deploy/choreo."));
        chooser.addRoutine("TUNE_MOI", () -> tuneMoi(autos));
        chooser.addCmd("THE_LAMEST_AUTO_EVER", Trajectories::theLamestAutoEver);
        chooser.addRoutine("trenchSSOutpost", () -> trenchSsOutpost(autos));
        chooser.addRoutine("trenchSSDepot", () -> trenchSsDepot(autos));
    }

    private static AutoRoutine basicShoot(Autos autos) {
        AutoRoutine routine = autos.routine("basicShoot");
        AutoTrajectory[] paths = autos.split(routine, ChoreoTraj.basicShoot);

        IntakePivot intakePivot = autos.getRegistry().get(IntakePivot.class).get();
        Shooter shooter = autos.getRegistry().get(Shooter.class).get();

        autos.start(
                routine,
                paths[0],
                Commands.race(intakePivot.deploy(),
                        Commands.waitSeconds(BASIC_SHOOT_START_DELAY_SECS)));
        autos.then(paths[0], paths[1], Commands.race(Commands.waitSeconds(BASIC_SHOOT_SHOOT_TIME_SECS), shooter.shoot()));
        autos.then(paths[1], paths[2], logStep("TODO port climber.deploy()"));
        autos.finish(paths[2], logStep("TODO port climber.climb()"), autos.stopCommand());

        return routine;
    }

    private static AutoRoutine tuneMoi(Autos autos) {
        AutoRoutine routine = autos.routine("TUNE_MOI");
        AutoTrajectory path = autos.trajectory(routine, ChoreoTraj.CalibrateMOI);

        autos.start(routine, path);
        autos.finish(path, autos.stopCommand());

        return routine;
    }

    private static AutoRoutine trenchSsOutpost(Autos autos) {
        AutoRoutine routine = autos.routine("trenchSSOutpost");
        AutoTrajectory[] paths = autos.split(routine, ChoreoTraj.trenchSSOutpost);

        autos.start(
                routine,
                paths[0],
                logStep("TODO port intakePivot.deploy()"),
                logStep("TODO port intakeRollers.intakeForever() while trenchSSOutpost$0 is active"));
        autos.then(paths[0], paths[1], logStep("TODO port intakeRollers.stopIntake()"));
        autos.then(paths[1], paths[2], timedLogStep("TODO port shootSequence()", OUTPOST_SHOOT_TIME_SECS));
        autos.chain(paths[2], paths[3]);
        autos.then(paths[3], paths[4], Commands.waitSeconds(OUTPOST_BALL_DROP_WAIT_SECS));
        autos.chain(paths[4], paths[5]);
        autos.then(paths[5], paths[6], timedLogStep("TODO port shootSequence()", OUTPOST_SHOOT_TIME_SECS));
        autos.then(paths[6], paths[7], logStep("TODO port climber.deploy()"));
        autos.finish(paths[7], logStep("TODO port climber.climb()"), autos.stopCommand());

        return routine;
    }

    private static AutoRoutine trenchSsDepot(Autos autos) {
        AutoRoutine routine = autos.routine("trenchSSDepot");
        AutoTrajectory[] paths = autos.split(routine, ChoreoTraj.trenchSSDepot);

        autos.start(
                routine,
                paths[0],
                logStep("TODO port intakePivot.deploy()"),
                logStep("TODO port intakeRollers.intake() during trenchSSDepot collection"));
        autos.chain(paths[0], paths[1], paths[2], paths[3], paths[4]);
        autos.finish(paths[4], timedLogStep("TODO port shootSequence()", DEPOT_SHOOT_TIME_SECS), autos.stopCommand());

        return routine;
    }

    private static Command theLamestAutoEver() {
        return Commands.sequence(
                logStep("TODO port intakePivot.deploy()"),
                Commands.waitSeconds(2.0),
                timedLogStep("TODO port shooter/indexer sequence from THE_LAMEST_AUTO_EVER()",
                        LAMEST_AUTO_TOTAL_TIME_SECS - 2.0));
    }

    private static Command unavailableAuto(String autoName, String reason) {
        return Commands.print("Auto '" + autoName + "' is unavailable: " + reason);
    }

    private static Command logStep(String message) {
        return Commands.print("[Auto] " + message);
    }

    private static Command timedLogStep(String message, double seconds) {
        return Commands.sequence(logStep(message), Commands.waitSeconds(seconds));
    }
}
