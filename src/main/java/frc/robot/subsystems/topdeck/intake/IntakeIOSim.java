package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.Intake_Down_SetPoint;
import static frc.robot.Constants.IntakeConstants.Intake_Up_SetPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements intakeIO {
  private static final DCMotor gearbox = DCMotor.getKrakenX60(1);
  private static final double moi = 0.004;
  private static final double reduction = 1.0;

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, moi, reduction), gearbox);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(intakeIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    double positionRot = Units.radiansToRotations(sim.getAngularPositionRad());
    double velocityRPM =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    double currentAmps = Math.abs(sim.getCurrentDrawAmps());

    inputs.intakeConnected = true;
    inputs.intakePositionRot = positionRot;
    inputs.intakeVelocityRPM = velocityRPM;
    inputs.intakeAppliedVolts = appliedVolts;
    inputs.intakeSupplyCurrentAmps = currentAmps;
    inputs.intakeStatorCurrentAmps = currentAmps;
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryIntakePositionsRot = new double[] {positionRot};
    inputs.odometryIntakeVelocityRPM = new double[] {velocityRPM};

    inputs.intakeKrakenPositionRot = positionRot;
    inputs.intakeKrakenVelocityRPM = velocityRPM;
    inputs.intakeKrakenAppliedVolts = appliedVolts;
    inputs.intakeKrakenSupplyCurrentAmps = currentAmps;
    inputs.intakeKrakenStatorCurrentAmps = currentAmps;
    inputs.odometryKrakenTimestamps = new double[] {inputs.odometryTimestamps[0]};
    inputs.odometryKrakenPositionsRot = new double[] {positionRot};
    inputs.odometryKrakenVelocityRPM = new double[] {velocityRPM};
  }

  @Override
  public void setIntakeOpenLoop(double output) {
    appliedVolts = output * 12.0;
  }

  @Override
  public boolean setIntakeOpenLoopUntilSetpointDown() {
    setIntakeOpenLoop(-12);
    return Units.radiansToRotations(sim.getAngularPositionRad()) <= Intake_Down_SetPoint;
  }

  @Override
  public boolean setIntakeOpenLoopUntilSetpointUp() {
    setIntakeOpenLoop(12);
    return Units.radiansToRotations(sim.getAngularPositionRad()) >= Intake_Up_SetPoint;
  }

  @Override
  public void setKrakenOpenLoop(double output) {
    appliedVolts = output * 12.0;
  }

  @Override
  public void setKrakenVoltage(double volts) {
    appliedVolts = volts;
  }
}
