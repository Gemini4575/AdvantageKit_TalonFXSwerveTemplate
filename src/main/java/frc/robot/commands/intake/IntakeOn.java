package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class IntakeOn extends Command {
  private Intake i;

  public IntakeOn(Intake i) {
    this.i = i;
    addRequirements(i);
  }

  @Override
  public void execute() {
    i.intake();
  }

  @Override
  public void end(boolean interrupted) {
    i.stopIntake();
  }
}
