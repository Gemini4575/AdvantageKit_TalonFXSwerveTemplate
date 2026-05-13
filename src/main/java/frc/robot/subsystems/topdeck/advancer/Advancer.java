package frc.robot.subsystems.topdeck.advancer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AdvancerConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Advancer extends SubsystemBase {
  private final AdvancerMotor talonAdvancer;
  private final AdvancerMotor neoAdvancer;
  private final AdvancerMotor roller;
  private final SysIdRoutine talonAdvancerSysId;
  private final SysIdRoutine neoAdvancerSysId;
  private final SysIdRoutine rollerSysId;

  public Advancer(AdvancerIO talonAdvancerIO, AdvancerIO neoAdvancerIO, AdvancerIO rollerIO) {
    talonAdvancer = new AdvancerMotor(talonAdvancerIO, "talon");
    neoAdvancer = new AdvancerMotor(neoAdvancerIO, "neo");
    roller = new AdvancerMotor(rollerIO, "roller");

    talonAdvancerSysId = makeSysIdRoutine("Talon", this::runTalonCharacterization);
    neoAdvancerSysId = makeSysIdRoutine("Neo", this::runNeoCharacterization);
    rollerSysId = makeSysIdRoutine("Roller", this::runRollerCharacterization);
  }

  private SysIdRoutine makeSysIdRoutine(
      String name, java.util.function.DoubleConsumer voltageConsumer) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Advancer/" + name + "/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> voltageConsumer.accept(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    talonAdvancer.periodic();
    neoAdvancer.periodic();
    roller.periodic();

    if (DriverStation.isDisabled()) {
      stopAdvancer();
    }
  }

  public void reverse() {
    talonAdvancer.setOpenLoop(KRAKEN_ADCNAVER_SPEED);
    neoAdvancer.setOpenLoop(NEO_ADVANCER_SPEED);
    roller.setOpenLoop(ADVANCER_ROLLER_SPEED);
  }

  public void advancerOnlyReverse() {
    talonAdvancer.setOpenLoop(KRAKEN_ADCNAVER_SPEED);
    neoAdvancer.setOpenLoop(NEO_ADVANCER_SPEED);
  }

  public void stopAdvancer() {
    talonAdvancer.stop();
    neoAdvancer.stop();
    roller.stop();
  }

  public void advance() {
    talonAdvancer.setOpenLoop(-KRAKEN_ADCNAVER_SPEED);
    neoAdvancer.setOpenLoop(-NEO_ADVANCER_SPEED);
    roller.setOpenLoop(-ADVANCER_ROLLER_SPEED);
  }

  public void runTalonCharacterization(double volts) {
    talonAdvancer.setVoltage(volts);
    neoAdvancer.stop();
    roller.stop();
  }

  public void runNeoCharacterization(double volts) {
    talonAdvancer.stop();
    neoAdvancer.setVoltage(volts);
    roller.stop();
  }

  public void runRollerCharacterization(double volts) {
    talonAdvancer.stop();
    neoAdvancer.stop();
    roller.setVoltage(volts);
  }

  public Command talonSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(this::stopAdvancer)
        .withTimeout(1.0)
        .andThen(talonAdvancerSysId.quasistatic(direction));
  }

  public Command talonSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(this::stopAdvancer).withTimeout(1.0).andThen(talonAdvancerSysId.dynamic(direction));
  }

  public Command neoSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(this::stopAdvancer)
        .withTimeout(1.0)
        .andThen(neoAdvancerSysId.quasistatic(direction));
  }

  public Command neoSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(this::stopAdvancer).withTimeout(1.0).andThen(neoAdvancerSysId.dynamic(direction));
  }

  public Command rollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(this::stopAdvancer).withTimeout(1.0).andThen(rollerSysId.quasistatic(direction));
  }

  public Command rollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(this::stopAdvancer).withTimeout(1.0).andThen(rollerSysId.dynamic(direction));
  }
}
