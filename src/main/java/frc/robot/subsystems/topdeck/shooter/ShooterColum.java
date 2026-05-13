package frc.robot.subsystems.topdeck.shooter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class ShooterColum {
  private final ShooterColumIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final String name;

  private final Alert ShooterDisconnectedAlert;

  public ShooterColum(ShooterColumIO io, int index) {
    this.io = io;
    name = "column" + index;
    ShooterDisconnectedAlert =
        new Alert(
            "Disconnected shooter motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/" + name, inputs);

    // Update alerts
    ShooterDisconnectedAlert.set(!inputs.shooterConnected);
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public void stop() {
    io.setShooterOpenLoop(0.0);
  }

  public double[] getVelocity() {
    return inputs.odometryShooterVelocityRot;
  }

  public double[] getPosition() {
    return inputs.odometryShooterPositionsRad;
  }

  public void setOpenLoop(double output) {
    io.setShooterOpenLoop(output);
  }

  public void setVelocity(double velocityRotationsPerMin) {
    io.setShooterVelocity(velocityRotationsPerMin);
  }
}
