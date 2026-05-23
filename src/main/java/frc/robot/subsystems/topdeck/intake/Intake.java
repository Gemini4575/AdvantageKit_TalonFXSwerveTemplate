package frc.robot.subsystems.topdeck.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.States;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static final double TEST_ROTATOR_TOLERANCE_ROT = 2.0;
  private static final double TEST_MIN_ROLLER_MOVEMENT_ROT = 0.1;

  private final intakeIO rotatorIO;
  private final intakeIO rollerIO;
  private final intakeIOInputsAutoLogged rotatorInputs = new intakeIOInputsAutoLogged();
  private final intakeIOInputsAutoLogged rollerInputs = new intakeIOInputsAutoLogged();
  private double rollerTestStartRot;

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

    if (States.INTAKE_ON_AUTON && DriverStation.isAutonomous()) {
      intake();
    }

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

  public boolean printHealth() {
    boolean rotatorOk = rotatorInputs.intakeConnected;
    boolean rollerOk = rollerInputs.intakeConnected;

    System.out.println("Intake:");
    printMotorHealth("  Rotator", rotatorOk);
    printMotorHealth("  Roller", rollerOk);
    return rotatorOk && rollerOk;
  }

  public Command motorResponseTest() {
    return Commands.sequence(
            Commands.runOnce(
                () ->
                    System.out.printf(
                        "Intake function check: down=%.1f rot, up=%.1f rot, tolerance=+/-%.1f rot%n",
                        Intake_Down_SetPoint, Intake_Up_SetPoint, TEST_ROTATOR_TOLERANCE_ROT)),
            Commands.run(this::moveDownToIntake, this)
                .until(this::isRotatorNearDown)
                .withTimeout(3.0),
            Commands.runOnce(
                () -> printRotatorPositionHealth("  Intake down", Intake_Down_SetPoint)),
            Commands.run(this::moveUpToStore, this).until(this::isRotatorNearUp).withTimeout(3.0),
            Commands.runOnce(() -> printRotatorPositionHealth("  Intake up", Intake_Up_SetPoint)),
            Commands.runOnce(() -> rollerTestStartRot = rollerInputs.intakeKrakenPositionRot),
            Commands.run(this::intake, this).withTimeout(0.75),
            Commands.runOnce(this::printRollerResponseHealth))
        .finallyDo(this::stop);
  }

  private boolean isRotatorNearDown() {
    return Math.abs(rotatorInputs.intakePositionRot - Intake_Down_SetPoint)
        <= TEST_ROTATOR_TOLERANCE_ROT;
  }

  private boolean isRotatorNearUp() {
    return Math.abs(rotatorInputs.intakePositionRot - Intake_Up_SetPoint)
        <= TEST_ROTATOR_TOLERANCE_ROT;
  }

  private void printRotatorPositionHealth(String name, double targetRot) {
    double positionRot = rotatorInputs.intakePositionRot;
    double errorRot = Math.abs(positionRot - targetRot);
    boolean good = rotatorInputs.intakeConnected && errorRot <= TEST_ROTATOR_TOLERANCE_ROT;

    System.out.printf(
        "%s position: %s (target=%.1f rot, measured=%.1f rot, error=%.1f rot)%n",
        name, good ? "GOOD" : "BAD", targetRot, positionRot, errorRot);
  }

  private void printRollerResponseHealth() {
    double appliedVolts = Math.abs(rollerInputs.intakeKrakenAppliedVolts);
    double positionRot = rollerInputs.intakeKrakenPositionRot;
    double movementRot = Math.abs(positionRot - rollerTestStartRot);
    boolean good = rollerInputs.intakeConnected && movementRot >= TEST_MIN_ROLLER_MOVEMENT_ROT;

    System.out.printf(
        "  Intake roller movement: %s (moved=%.2f rot, start=%.2f rot, end=%.2f rot,"
            + " applied=%.2f V)%n",
        good ? "GOOD" : "BAD", movementRot, rollerTestStartRot, positionRot, appliedVolts);
  }

  private static void printMotorHealth(String name, boolean good) {
    System.out.println(name + ": " + (good ? "GOOD" : "BAD"));
  }
}
