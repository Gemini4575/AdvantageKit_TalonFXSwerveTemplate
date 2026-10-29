package frc.robot.subsystems.topdeck.elevator;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** High-level control and AdvantageKit logging for the elevator. */
public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public double getVelocityMetersPerSecond() {
    return inputs.velocityMetersPerSec;
  }

  public double getMinHeightMeters() {
    return minHeightMeters;
  }

  public double getMaxHeightMeters() {
    return maxHeightMeters;
  }

  public void setPosition(double positionMeters) {
    io.setPosition(
        MathUtil.clamp(positionMeters, minHeightMeters, maxHeightMeters), feedforward.calculate(0.0));
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.stop();
  }

  public Command setHeightCommand(double heightMeters) {
    return run(() -> setPosition(heightMeters));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
