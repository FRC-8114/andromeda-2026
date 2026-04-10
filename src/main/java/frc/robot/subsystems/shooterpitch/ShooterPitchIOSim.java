package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterPitchIOSim implements ShooterPitchIO {
    private static final DCMotor GEARBOX     = DCMotor.getKrakenX60Foc(1);
    private static final double  GEAR_RATIO  = 12.67;
    private static final double  MOI_KG_M2   = 0.03;
    private static final double  ARM_LENGTH_M = Meters.of(0.15).magnitude();

    private static final double KP = 2.0;
    private static final double KD = 0.2;

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        GEARBOX,
        GEAR_RATIO,
        MOI_KG_M2,
        ARM_LENGTH_M,
        ShooterPitch.Constants.MIN_ANGLE.in(Radians),
        ShooterPitch.Constants.MAX_ANGLE.in(Radians),
        true,
        ShooterPitch.Constants.MIN_ANGLE.in(Radians));

    private final PIDController controller = new PIDController(KP, 0, KD);

    private boolean closedLoop = false;
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    private Angle targetAngle = ShooterPitch.Constants.MIN_ANGLE;

    @Override
    public void setTarget(Angle angle) {
        targetAngle = angle;
        controller.setSetpoint(angle.in(Radians));
        closedLoop = true;
    }

    @Override
    public void setVoltage(Voltage volts) {
        closedLoop = false;
        appliedVoltage.mut_replace(volts);
    }

    @Override
    public void runCurrent(Current current) {
        // Approximate: V = I * R
        closedLoop = false;
        appliedVoltage.mut_replace(
            MathUtil.clamp(current.in(Amps) * GEARBOX.rOhms, -12.0, 12.0), Volts);
    }

    @Override
    public void stop() {
        closedLoop = false;
        appliedVoltage.mut_replace(0, Volts);
    }

    @Override
    public void updateInputs(ShooterPitchInputs inputs) {
        if (closedLoop) {
            appliedVoltage.mut_replace(
                controller.calculate(sim.getAngleRads()), Volts);
        } else {
            controller.reset();
        }

        appliedVoltage.mut_replace(
            MathUtil.clamp(appliedVoltage.in(Volts), -12.0, 12.0), Volts);
        sim.setInputVoltage(appliedVoltage.in(Volts));
        sim.update(0.02);

        inputs.targetAngle.mut_replace(targetAngle);
        inputs.position.mut_replace(sim.getAngleRads(), Radians);
        inputs.velocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);
        inputs.appliedVoltage.mut_replace(appliedVoltage);
        inputs.appliedCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps);
    }
}
