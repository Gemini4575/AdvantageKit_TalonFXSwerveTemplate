package frc.robot.subsystems.topdeck.advancer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class AdvancerMotor {
  private final AdvancerIO io;
  private final AdvancerIOInputsAutoLogged inputs = new AdvancerIOInputsAutoLogged();
  private final String name;
  private final Alert disconnectedAlert;

  public AdvancerMotor(AdvancerIO io, String name) {
    this.io = io;
    this.name = name;
    disconnectedAlert = new Alert("Disconnected advancer motor: " + name + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Advancer/" + name, inputs);
    disconnectedAlert.set(!inputs.advancerConnected);
  }

  public void setOpenLoop(double output) {
    io.setAdvancerOpenLoop(output);
  }

  public void setVoltage(double volts) {
    io.setAdvancerVoltage(volts);
  }

  public void setVelocity(double velocityRotationsPerMin) {
    io.setAdvancerVelocity(velocityRotationsPerMin);
  }

  public void stop() {
    io.setAdvancerOpenLoop(0.0);
  }
}
