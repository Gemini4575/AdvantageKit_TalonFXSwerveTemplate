package frc.robot.commands.shooter;

import frc.robot.commands.shooter.base.TelopShootBase;
import frc.robot.subsystems.topdeck.advancer.Advancer;
import frc.robot.subsystems.topdeck.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShootFromHubTele extends TelopShootBase {
  public ShootFromHubTele(Shooter s, Advancer ds, DoubleSupplier velocitySupplier) {
    super(s, ds, velocitySupplier);
  }
}
