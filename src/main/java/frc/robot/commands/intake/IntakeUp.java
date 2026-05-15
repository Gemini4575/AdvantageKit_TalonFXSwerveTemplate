package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class IntakeUp extends Command {
  private final Intake i;
  private boolean isFinished;

  public IntakeUp(Intake ii) {
    i = ii;
    isFinished = false;
    addRequirements(i);
  }

  @Override
  public void execute() {
    if (i.moveUpToStore()) {
      isFinished = true;
    }
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
