package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;

import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final StatusSignal<Angle> pitch = pigeon.getPitch();
    private final StatusSignal<Angle> roll = pigeon.getRoll();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
    private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityYWorld();
    private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityXWorld();

    public GyroIOPigeon2() {
        if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
            pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
        } else {
            pigeon.getConfigurator().apply(new Pigeon2Configuration());
        }

        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(50.0);
        pitchVelocity.setUpdateFrequency(50.0);
        rollVelocity.setUpdateFrequency(50.0);
        pitch.setUpdateFrequency(50.0);
        roll.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity)
                .equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromRadians(yaw.getValue().in(Radians));
        inputs.pitchPosition = Rotation2d.fromRadians(pitch.getValue().in(Radians));
        inputs.rollPosition = Rotation2d.fromRadians(roll.getValue().in(Radians));
        inputs.yawVelocityRadPerSec = yawVelocity.getValue();
        inputs.pitchVelocityRadPerSec = pitchVelocity.getValue();
        inputs.rollVelocityRadPerSec = rollVelocity.getValue();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
