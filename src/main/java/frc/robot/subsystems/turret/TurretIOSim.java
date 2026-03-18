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
    private Angle targetAngle = Radians.of(0.0);

    public TurretIOSim() {
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(GEARBOX, MOI, GEAR_RATIO),
                GEARBOX);
        double initialAngle = 0.5 * (Turret.Constants.MIN_ANGLE.in(Radians) + Turret.Constants.MAX_ANGLE.in(Radians));
        sim.setState(VecBuilder.fill(initialAngle, 0.0));
        targetAngle = Radians.of(initialAngle);
    }

    public void setTarget(Angle angle) {
        closedLoop = true;
        targetAngle = angle;
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
        inputs.goalPosition = targetAngle;
        inputs.currentTurretPosition = Radians.of(sim.getAngularPositionRad());
        inputs.crtTurretPosition = Radians.of(sim.getAngularPositionRad());
        inputs.motorPositionErrorCounter = 0;
        inputs.turretVelocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
        inputs.appliedVoltage = Volts.of(appliedVolts);
    }
}
