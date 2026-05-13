package frc.robot.subsystems.topdeck;

import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOSpark;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOSparkFlex;
import frc.robot.subsystems.topdeck.advancer.AdvancerIOTalonFX;

public class AdvancerSubsystem extends Advancer {
  public AdvancerSubsystem() {
    super(new AdvancerIOTalonFX(), new AdvancerIOSpark(), new AdvancerIOSparkFlex());
  }
}
