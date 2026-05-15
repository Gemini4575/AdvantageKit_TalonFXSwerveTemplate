package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class ExtendIntake extends Command {
  private Intake climber;

  public ExtendIntake(Intake climberSubsystem) {
    this.climber = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    climber.moveDownToIntake();
  }

  @Override
  public boolean isFinished() {
    return climber.moveDownToIntake();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
