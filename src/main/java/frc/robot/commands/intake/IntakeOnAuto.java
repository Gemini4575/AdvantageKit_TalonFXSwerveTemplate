package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class IntakeOnAuto extends Command {
  @Override
  public void initialize() {
    Constants.States.INTAKE_ON_AUTON = true;
  }
}
