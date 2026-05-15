package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class ExtendIntakeAndIntake extends Command {
  private final Intake i;

  public ExtendIntakeAndIntake(Intake ii) {
    i = ii;
    addRequirements(i);
  }

  @Override
  public void initialize() {
    i.moveDownToIntake();
  }

  @Override
  public void execute() {
    i.intake();
  }

  @Override
  public void end(boolean iii) {
    i.stopIntake();
  }
}
