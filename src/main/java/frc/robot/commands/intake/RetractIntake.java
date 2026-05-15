package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.topdeck.intake.Intake;

public class RetractIntake extends Command {
  private Intake climber;

  public RetractIntake(Intake climberSubsystem) {
    this.climber = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {
    climber.moveUpToStore();
  }

  @Override
  public boolean isFinished() {
    return climber.moveUpToStore();
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
