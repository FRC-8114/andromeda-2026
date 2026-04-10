package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollersIOSim implements IntakeRollersIO {
    private static final DCMotor ROLLER_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final double ROLLER_GEAR_RATIO = 1.0;
    private static final double ROLLER_MOI = 0.001;

    private static final double ROLLER_KP = 0.1;

    private final DCMotorSim rollerSim;

    private final PIDController rollerController = new PIDController(ROLLER_KP, 0, 0);
    private boolean rollerClosedLoop = false;

    private double rollerAppliedVolts = 0.0;
    private AngularVelocity targetVelocity = RadiansPerSecond.of(0);

    public IntakeRollersIOSim() {
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_GEAR_RATIO),
                ROLLER_GEARBOX);
    }

    public void setTargetVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        rollerController.setSetpoint(velocity.in(RadiansPerSecond));
        rollerClosedLoop = true;
    }

    public void runCurrent(Current current) {
        // Approximate: V = I * R
        rollerClosedLoop = false;
        rollerAppliedVolts = MathUtil.clamp(
            current.in(Amps) * ROLLER_GEARBOX.rOhms, -12.0, 12.0);
    }

    public void runVolts(Voltage volts) {
        rollerClosedLoop = false;
        rollerAppliedVolts = volts.in(Volts);
    }

    public void stop() {
        rollerClosedLoop = false;
        rollerAppliedVolts = 0.0;
    }
    
    public void updateInputs(IntakeRollersInputs inputs) {
        if (rollerClosedLoop) {
            rollerAppliedVolts = rollerController.calculate(rollerSim.getAngularVelocityRadPerSec());
        } else {
            rollerController.reset();
        }

        rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
        rollerSim.update(0.02);

        inputs.targetVelocityRadPerSec.mut_replace(targetVelocity);
        inputs.velocityRadPerSec.mut_replace(rollerSim.getAngularVelocity());
        inputs.appliedVoltageVolts.mut_replace(rollerAppliedVolts, Volts);
        inputs.positionRadians.mut_replace(rollerSim.getAngularPosition());
        inputs.currentAmps.mut_replace(rollerSim.getCurrentDrawAmps(), Amps);
    }
}
