package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public Rotation2d pitchPosition = Rotation2d.kZero;
        public Rotation2d rollPosition = Rotation2d.kZero;
        public AngularVelocity yawVelocity = RadiansPerSecond.of(0.0);
        public AngularVelocity pitchVelocity = RadiansPerSecond.of(0.0);
        public AngularVelocity rollVelocity = RadiansPerSecond.of(0.0);
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public void updateInputs(GyroIOInputs inputs);
}
