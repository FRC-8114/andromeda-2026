package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import java.nio.file.Files;
import java.nio.file.Path;
import org.littletonrobotics.junction.Logger;

public class RobotConstants {
    public enum RobotMode {
        REAL,
        SIMULATION,
        REPLAY
    };

    private static final String replayLogPathEnvVar = "AKIT_LOG_PATH";
    private static final String advantageScopeReplayPathFile = "akit-log-path.txt";

    public static RobotMode getRobotMode() {
        if (RobotBase.isReal()) {
            return RobotMode.REAL;
        }

        return isReplayRequested() ? RobotMode.REPLAY : RobotMode.SIMULATION;
    }

    private static boolean isReplayRequested() {
        if (Logger.hasReplaySource()) {
            return true;
        }

        if (System.getenv(replayLogPathEnvVar) != null) {
            return true;
        }

        Path advantageScopeReplayPath = Path.of(System.getProperty("java.io.tmpdir"), advantageScopeReplayPathFile);
        return Files.exists(advantageScopeReplayPath);
    }

    public static final CANBus canBus = TunerConstants.kCANBus;
}
