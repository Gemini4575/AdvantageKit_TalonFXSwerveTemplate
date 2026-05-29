package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class IntakeStopMovingAuto extends Command {
  private final Intake intake;

  public IntakeStopMovingAuto(Intake i) {
    intake = i;
    addRequirements(i);
  }

  @Override
  public void initialize() {
    intake.stopRotator();
  }
}
