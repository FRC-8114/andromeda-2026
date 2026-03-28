package frc.robot.auto;

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
    private static final double BASIC_SHOOT_START_DELAY_SECS = 2.0;
    private static final double BASIC_SHOOT_SHOOT_TIME_SECS = 6.0;
    private static final double OUTPOST_SHOOT_TIME_SECS = 5.0;
    private static final double OUTPOST_BALL_DROP_WAIT_SECS = 4.0;
    private static final double DEPOT_SHOOT_TIME_SECS = 5.0;
    private static final double LAMEST_AUTO_TOTAL_TIME_SECS = 7.0;

    private Trajectories() {}

    public static void addAutos(AutoChooser chooser, Autos autos) {
        chooser.addRoutine("TUNE_MOI", () -> tuneMoi(autos));
        chooser.addRoutine("Trench2xOutpost", () -> trench2xOutpost(autos));
    }

    private static AutoRoutine trench2xOutpost(Autos autos) {
        AutoRoutine routine = autos.routine("Trench2xOutpostSHORT");
        AutoTrajectory paths = autos.trajectory(routine, ChoreoTraj.Trench2xOutpostSHORT);

        autos.start(routine, paths);
        autos.finish(paths, autos.stopCommand());

        return routine;
    }

    private static AutoRoutine tuneMoi(Autos autos) {
        AutoRoutine routine = autos.routine("TUNE_MOI");
        AutoTrajectory path = autos.trajectory(routine, ChoreoTraj.CalibrateMOI);

        autos.start(routine, path);
        autos.finish(path, autos.stopCommand());

        return routine;
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
