package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class IntakeDown extends Command {
  private final Intake i;
  private boolean isFinished;

  public IntakeDown(Intake ii) {
    i = ii;
    isFinished = false;
    addRequirements(i);
  }

  @Override
  public void execute() {
    if (i.moveDownToIntake()) {
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
