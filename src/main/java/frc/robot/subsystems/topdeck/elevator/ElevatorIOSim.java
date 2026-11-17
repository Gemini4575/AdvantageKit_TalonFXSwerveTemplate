package frc.robot.subsystems.topdeck.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Physics simulation for the two-motor elevator gearbox. */
public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(2),
          gearRatio,
          carriageMassKg,
          drumRadiusMeters,
          minHeightMeters,
          maxHeightMeters,
          true,
          minHeightMeters);
  private final PIDController controller = new PIDController(kP, kI, kD);
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sim.update(0.02);
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps =
        new double[] {sim.getCurrentDrawAmps() / 2.0, sim.getCurrentDrawAmps() / 2.0};
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setPosition(double positionMeters, double feedforwardVolts) {
    setVoltage(controller.calculate(sim.getPositionMeters(), positionMeters) + feedforwardVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
