package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States;
import frc.robot.subsystems.topdeck.intake.Intake;
import java.util.function.BooleanSupplier;

public class ExtendOrRectactIntake extends Command {
  private final Intake i;
  private final BooleanSupplier extend;
  private final BooleanSupplier retract;
  private final BooleanSupplier intake;

  public ExtendOrRectactIntake(
      Intake ii, BooleanSupplier extend, BooleanSupplier retract, BooleanSupplier intake) {
    i = ii;
    addRequirements(i);
    this.extend = extend;
    this.retract = retract;
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (extend.getAsBoolean()) {
      i.moveDownToIntake();
      i.intake();
    } else if (retract.getAsBoolean()) {
      i.moveUpToStore();
    } else if (intake.getAsBoolean()) {
      i.outtake();
    } else if (DriverStation.isAutonomous() && States.INTAKE_ON_AUTON) {
      i.stopRotator();
    } else {
      i.stopIntake();
      i.stopRotator();
    }
  }

  @Override
  public void end(boolean interrupted) {
    i.stop();
  }
}
