package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final intakeIO rotatorIO;
  private final intakeIO rollerIO;
  private final intakeIOInputsAutoLogged rotatorInputs = new intakeIOInputsAutoLogged();
  private final intakeIOInputsAutoLogged rollerInputs = new intakeIOInputsAutoLogged();

  public Intake(intakeIO rotatorIO, intakeIO rollerIO) {
    this.rotatorIO = rotatorIO;
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    rotatorIO.updateInputs(rotatorInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Rotator", rotatorInputs);
    Logger.processInputs("Intake/Rollers", rollerInputs);

    if (DriverStation.isDisabled()) {
      stop();
    }
  }

  public void intake() {
    Constants.States.INTAKE_ON = true;
    rollerIO.setKrakenOpenLoop(INTAKE_SPEED);
  }

  public void outtake() {
    Constants.States.INTAKE_ON = true;
    rollerIO.setKrakenOpenLoop(-INTAKE_SPEED);
  }

  public void stopIntake() {
    Constants.States.INTAKE_ON = false;
    rollerIO.setKrakenOpenLoop(0.0);
  }

  public void stopRotator() {
    rotatorIO.setIntakeOpenLoop(0.0);
  }

  public void stop() {
    stopIntake();
    stopRotator();
  }

  public boolean moveDownToIntake() {
    if (rotatorIO.setIntakeOpenLoopUntilSetpointDown()) {
      stopRotator();
      Constants.States.INTAKE_IN = false;
      return true;
    }
    return false;
  }

  public boolean moveUpToStore() {
    if (rotatorIO.setIntakeOpenLoopUntilSetpointUp()) {
      stopRotator();
      Constants.States.INTAKE_IN = true;
      return true;
    }
    return false;
  }

  public void testRotator(double output) {
    rotatorIO.setIntakeOpenLoop(output);
  }
}
