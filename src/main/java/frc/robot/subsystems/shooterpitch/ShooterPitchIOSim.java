package frc.robot.subsystems.shooterpitch;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterPitchIOSim implements ShooterPitchIO {
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double GEAR_RATIO = 1.2;
    private static final double MOI = 0.01; // kg*m^2, estimate for pitch mechanism

    private static final double KP = 2.0;
    private static final double KD = 0.2;

    private final DCMotorSim sim;
    private final PIDController controller = new PIDController(KP, 0, KD);

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public ShooterPitchIOSim() {
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEAR_RATIO),
                GEARBOX);
    }

    public void setTarget(Angle angle) {
        closedLoop = true;
        controller.setSetpoint(angle.in(Radians));
    }

    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
    }

    public void updateInputs(ShooterPitchInputs inputs) {
        if (closedLoop) {
            appliedVolts = controller.calculate(sim.getAngularPositionRad());
        } else {
            controller.reset();
        }

        sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        sim.update(0.02);

        inputs.pitchPosition = sim.getAngularPositionRad();
        inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVoltage = appliedVolts;
    }
}
