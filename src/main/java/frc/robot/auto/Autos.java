package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SubsystemRegistry;
import frc.robot.util.SysIDMechanism;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
public class Autos {
    public record Branch(BooleanSupplier condition, List<ChoreoTraj> trajectories) {
        public Branch {
            trajectories = List.copyOf(trajectories);
            if (trajectories.isEmpty()) {
                throw new IllegalArgumentException("Branch must contain at least one trajectory");
            }
        }
    }

    private record TrajectoryChain(AutoTrajectory firstTrajectory, AutoTrajectory lastTrajectory) {
    }

    private final SubsystemRegistry subsystemRegistry;
    private final Drive drive;
    private final AutoFactory autoFactory;

    public Autos(SubsystemRegistry subsystemRegistry) {
        this.subsystemRegistry = subsystemRegistry;
        drive = subsystemRegistry.require(Drive.class);
        autoFactory = new AutoFactory(
                drive::getPose,
                drive::setPose,
                drive::followTrajectory,
                true,
                drive,
                Autos::logTrajectory);
    }

    public static Branch when(
            BooleanSupplier condition, ChoreoTraj firstTrajectory, ChoreoTraj... remainingTrajectories) {
        return new Branch(condition, toTrajectoryList(firstTrajectory, remainingTrajectories));
    }

    public static Branch otherwise(ChoreoTraj firstTrajectory, ChoreoTraj... remainingTrajectories) {
        return when(() -> true, firstTrajectory, remainingTrajectories);
    }

    public AutoChooser createChooser() {
        AutoChooser chooser = new AutoChooser();
        Trajectories.addAutos(chooser, this, subsystemRegistry);
        addSysIdCommands(chooser);
        return chooser;
    }

    public SubsystemRegistry getRegistry() {
        return subsystemRegistry;
    }

    public List<SysIDMechanism.NamedMechanism> sysIdMechanisms() {
        List<SysIDMechanism.NamedMechanism> mechanisms = subsystemRegistry.all(SysIDMechanism.class).stream()
                .flatMap(provider -> provider.sysIdMechanisms().stream())
                .toList();

        Set<String> names = new HashSet<>();
        for (SysIDMechanism.NamedMechanism mechanism : mechanisms) {
            if (!names.add(mechanism.name())) {
                throw new IllegalStateException("Duplicate SysId mechanism name: " + mechanism.name());
            }
        }

        return mechanisms;
    }

    public AutoRoutine routine(String routineName) {
        return autoFactory.newRoutine(routineName);
    }

    public AutoTrajectory trajectory(AutoRoutine routine, ChoreoTraj trajectoryConfig) {
        return trajectoryConfig.asAutoTraj(routine);
    }

    public AutoTrajectory segment(AutoRoutine routine, ChoreoTraj trajectoryConfig, int segmentIndex) {
        return trajectoryConfig.segment(segmentIndex).asAutoTraj(routine);
    }

    public AutoTrajectory[] split(AutoRoutine routine, ChoreoTraj trajectoryConfig) {
        return expandTrajectory(trajectoryConfig).stream()
                .map(expandedTrajectory -> expandedTrajectory.asAutoTraj(routine))
                .toArray(AutoTrajectory[]::new);
    }

    public void start(AutoRoutine routine, AutoTrajectory trajectory, Command... beforeTrajectory) {
        routine.active().onTrue(
                Commands.sequence(
                        trajectory.resetOdometry(),
                        parallel(beforeTrajectory),
                        trajectory.cmd()));
    }

    public void then(AutoTrajectory currentTrajectory, AutoTrajectory nextTrajectory, Command... betweenTrajectories) {
        currentTrajectory.done().onTrue(
                Commands.sequence(
                        parallel(betweenTrajectories),
                        nextTrajectory.cmd()));
    }

    public void chain(AutoTrajectory firstTrajectory, AutoTrajectory... remainingTrajectories) {
        AutoTrajectory previousTrajectory = firstTrajectory;
        for (AutoTrajectory trajectory : remainingTrajectories) {
            previousTrajectory.done().onTrue(trajectory.cmd());
            previousTrajectory = trajectory;
        }
    }

    public void finish(AutoTrajectory trajectory, Command... afterTrajectory) {
        trajectory.done().onTrue(sequence(afterTrajectory));
    }

    public AutoRoutine follow(ChoreoTraj trajectoryConfig) {
        return chain(trajectoryConfig.name(), trajectoryConfig);
    }

    public AutoRoutine chain(
            String routineName, ChoreoTraj firstTrajectory, ChoreoTraj... remainingTrajectories) {
        AutoRoutine routine = autoFactory.newRoutine(routineName);
        TrajectoryChain trajectoryChain = createTrajectoryChain(routine,
                toTrajectoryList(firstTrajectory, remainingTrajectories));

        routine
                .active()
                .onTrue(
                        Commands.sequence(
                                trajectoryChain.firstTrajectory().resetOdometry(),
                                trajectoryChain.firstTrajectory().cmd()));
        trajectoryChain.lastTrajectory().done().onTrue(stopCommand());

        return routine;
    }

    public AutoRoutine branch(
            String routineName, ChoreoTraj entryTrajectory, Branch firstBranch, Branch... remainingBranches) {
        AutoRoutine routine = autoFactory.newRoutine(routineName);
        TrajectoryChain entryChain = createTrajectoryChain(routine, List.of(entryTrajectory));
        List<Branch> branches = new ArrayList<>();
        branches.add(firstBranch);
        branches.addAll(List.of(remainingBranches));

        routine
                .active()
                .onTrue(Commands.sequence(entryChain.firstTrajectory().resetOdometry(),
                        entryChain.firstTrajectory().cmd()));

        for (int i = 0; i < branches.size(); i++) {
            TrajectoryChain branchChain = createTrajectoryChain(routine, branches.get(i).trajectories());
            entryChain
                    .lastTrajectory()
                    .done()
                    .and(firstTrueBranchCondition(branches, i))
                    .onTrue(branchChain.firstTrajectory().cmd());
            branchChain.lastTrajectory().done().onTrue(stopCommand());
        }

        entryChain
                .lastTrajectory()
                .done()
                .and(() -> branches.stream().noneMatch(branch -> branch.condition().getAsBoolean()))
                .onTrue(stopCommand());

        return routine;
    }

    private static List<ChoreoTraj> toTrajectoryList(
            ChoreoTraj firstTrajectory, ChoreoTraj... remainingTrajectories) {
        List<ChoreoTraj> trajectories = new ArrayList<>();
        trajectories.add(firstTrajectory);
        trajectories.addAll(List.of(remainingTrajectories));
        return trajectories;
    }

    private static Command sequence(Command... commands) {
        return switch (commands.length) {
            case 0 -> Commands.none();
            case 1 -> commands[0];
            default -> Commands.sequence(commands);
        };
    }
    private static Command parallel(Command... commands) {
        return switch (commands.length) {
            case 0 -> Commands.none();
            case 1 -> commands[0];
            default -> Commands.parallel(commands);
        };
    }

    private TrajectoryChain createTrajectoryChain(AutoRoutine routine, List<ChoreoTraj> trajectoryConfigs) {
        List<AutoTrajectory> trajectories = new ArrayList<>();
        for (ChoreoTraj trajectoryConfig : trajectoryConfigs) {
            for (ChoreoTraj expandedTrajectory : expandTrajectory(trajectoryConfig)) {
                trajectories.add(expandedTrajectory.asAutoTraj(routine));
            }
        }

        for (int i = 0; i < trajectories.size() - 1; i++) {
            trajectories.get(i).done().onTrue(trajectories.get(i + 1).cmd());
        }

        return new TrajectoryChain(trajectories.get(0), trajectories.get(trajectories.size() - 1));
    }

    private List<ChoreoTraj> expandTrajectory(ChoreoTraj trajectoryConfig) {
        if (trajectoryConfig.segment().isPresent()) {
            return List.of(trajectoryConfig);
        }

        List<ChoreoTraj> splitSegments = ChoreoTraj.ALL_TRAJECTORIES.values().stream()
                .filter(candidate -> trajectoryConfig.name().equals(candidate.name()))
                .filter(candidate -> candidate.segment().isPresent())
                .sorted(Comparator.comparingInt(candidate -> candidate.segment().orElseThrow()))
                .toList();
        return splitSegments.isEmpty() ? List.of(trajectoryConfig) : splitSegments;
    }

    private BooleanSupplier firstTrueBranchCondition(List<Branch> branches, int index) {
        return () -> {
            for (int earlierBranch = 0; earlierBranch < index; earlierBranch++) {
                if (branches.get(earlierBranch).condition().getAsBoolean()) {
                    return false;
                }
            }
            return branches.get(index).condition().getAsBoolean();
        };
    }

    public Command stopCommand() {
        return Commands.runOnce(drive::stop, drive);
    }

    private void addSysIdCommands(AutoChooser chooser) {
        for (SysIDMechanism.NamedMechanism mechanism : sysIdMechanisms()) {
            chooser.addCmd(
                    sysIdCommandName(mechanism, "Quasistatic Forward"),
                    () -> mechanism.quasistatic().apply(SysIdRoutine.Direction.kForward));
            chooser.addCmd(
                    sysIdCommandName(mechanism, "Quasistatic Reverse"),
                    () -> mechanism.quasistatic().apply(SysIdRoutine.Direction.kReverse));
            chooser.addCmd(
                    sysIdCommandName(mechanism, "Dynamic Forward"),
                    () -> mechanism.dynamic().apply(SysIdRoutine.Direction.kForward));
            chooser.addCmd(
                    sysIdCommandName(mechanism, "Dynamic Reverse"),
                    () -> mechanism.dynamic().apply(SysIdRoutine.Direction.kReverse));
        }
    }

    private static String sysIdCommandName(SysIDMechanism.NamedMechanism mechanism, String testType) {
        return "SysId: " + mechanism.name() + " " + testType;
    }

    private static void logTrajectory(Trajectory<SwerveSample> trajectory, Boolean active) {
    }
}
