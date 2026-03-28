package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
    private static final DCMotor CLIMBER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double CLIMBER_GEAR_RATIO = 20.0;
    private static final double CLIMBER_MOI = 0.01;

    private static final double CLIMBER_KP = 5.0;

    private final DCMotorSim climberSim;
    private final PIDController climberController = new PIDController(CLIMBER_KP, 0, 0);

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public ClimberIOSim() {
        climberSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(CLIMBER_GEARBOX, CLIMBER_MOI, CLIMBER_GEAR_RATIO),
                CLIMBER_GEARBOX);
        
        climberController.setSetpoint(0);
    }

    @Override
    public void runVolts(Voltage volts) {
        closedLoop = false;
        appliedVolts = volts.in(Volts);
    }

    public void setPosition(double rotations) {
        closedLoop = true;
        climberController.setSetpoint(rotations * 2.0 * Math.PI);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = climberController.calculate(climberSim.getAngularPositionRad());
        } else {
            climberController.reset();
        }

        climberSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
        climberSim.update(0.02);

        inputs.positionRot = climberSim.getAngularPositionRad() / (2.0 * Math.PI);
        inputs.velocityRPM = climberSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI) * 60.0;
        inputs.appliedVoltageVolts = appliedVolts;
        inputs.currentAmps = Math.abs(climberSim.getCurrentDrawAmps());
    }
}
