package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private static final DCMotor GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double GEAR_RATIO = 20.0;
    private static final double MOI = 0.025; // kg*m^2, estimate for turret pivot

    private static final double KP = 5.0;
    private static final double KD = 0.0;

    private final DCMotorSim sim;
    private final PIDController controller = new PIDController(KP, 0, KD);

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public TurretIOSim() {
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEAR_RATIO),
                GEARBOX);
        double initialAngle = 0.5 * (Turret.Constants.MIN_ANGLE.in(Radians) + Turret.Constants.MAX_ANGLE.in(Radians));
        sim.setState(VecBuilder.fill(initialAngle, 0.0));
        controller.setSetpoint(initialAngle);
    }

    public void setTarget(Angle angle) {
        closedLoop = true;
        controller.setSetpoint(angle.in(Radians));
    }

    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
    }

    public void updateInputs(TurretIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = controller.calculate(sim.getAngularPositionRad());
        } else {
            controller.reset();
        }

        sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        sim.update(0.02);

        inputs.hasValidCRT = true;
        inputs.positionRadians.mut_replace(sim.getAngularPositionRad(), Radians);
        inputs.crtPositionRadians.mut_replace(sim.getAngularPositionRad(), Radians);
        inputs.crtPositionErrorRadians = 0.0;
        inputs.reseedSampleCount = 0;
        inputs.velocityRadPerSec.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        inputs.voltageVolts.mut_replace(appliedVolts, Volts);
    }

    @Override
    public void setCurrent(double amps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrent'");
    }
}
