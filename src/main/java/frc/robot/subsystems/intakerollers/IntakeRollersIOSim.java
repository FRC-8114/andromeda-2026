package frc.robot.subsystems.intakerollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

    public IntakeRollersIOSim() {
        rollerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(ROLLER_GEARBOX, ROLLER_MOI, ROLLER_GEAR_RATIO),
                ROLLER_GEARBOX);
    }

    public void runVolts(Voltage volts) {
        rollerClosedLoop = false;
        rollerAppliedVolts = volts.in(Volts);
    }

    @Override
    public void updateInputs(IntakeRollersInputs inputs) {
        if (rollerClosedLoop) {
            rollerAppliedVolts = rollerController.calculate(rollerSim.getAngularVelocityRadPerSec());
        } else {
            rollerController.reset();
        }

        rollerSim.setInputVoltage(MathUtil.clamp(rollerAppliedVolts, -12.0, 12.0));
        rollerSim.update(0.02);

        inputs.velocityRPM = rollerSim.getAngularVelocityRPM();
        inputs.appliedVoltageVolts = rollerAppliedVolts;
        inputs.rollerPositionRads = rollerSim.getAngularPositionRad();
    }

    @Override
    public void stopRollers() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopRollers'");
    }
}
