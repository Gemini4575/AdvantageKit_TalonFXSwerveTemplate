package frc.robot.subsystems.topdeck.elevator;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for the elevator gearbox. Positions and velocities are carriage values. */
public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderConnected = false;
    public boolean followerConnected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double[] tempCelsius = new double[] {};
  }

  /** Updates the set of loggable measurements from the elevator. */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Runs the elevator gearbox open-loop at the requested voltage. */
  default void setVoltage(double volts) {}

  /** Runs the elevator to a carriage-height target with the supplied feedforward voltage. */
  default void setPosition(double positionMeters, double feedforwardVolts) {}

  /** Stops the elevator gearbox. */
  default void stop() {}
}
