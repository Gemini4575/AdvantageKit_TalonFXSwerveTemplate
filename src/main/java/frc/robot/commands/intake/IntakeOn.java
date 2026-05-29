package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States;
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
    if (!DriverStation.isAutonomous() || !States.INTAKE_ON_AUTON) {
      i.stopIntake();
    }
  }
}
