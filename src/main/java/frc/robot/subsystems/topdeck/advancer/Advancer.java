package frc.robot.subsystems.topdeck.advancer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AdvancerConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Advancer extends SubsystemBase {
  private static final double FF_START_DELAY_SECS = 1.0;
  private static final double FF_RAMP_RATE_VOLTS_PER_SEC = 0.25;
  private static final double FF_PRINT_PERIOD_SECS = 0.25;

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
    roller.setOpenLoop(-12);
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
    roller.setOpenLoop(12);
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

  public Command talonConsoleSysId() {
    return consoleFeedforwardCharacterization(
        "Talon", talonAdvancer, this::runTalonCharacterization);
  }

  public Command neoConsoleSysId() {
    return consoleFeedforwardCharacterization("Neo", neoAdvancer, this::runNeoCharacterization);
  }

  public Command rollerConsoleSysId() {
    return consoleFeedforwardCharacterization("Roller", roller, this::runRollerCharacterization);
  }

  private Command consoleFeedforwardCharacterization(
      String name, AdvancerMotor motor, java.util.function.DoubleConsumer voltageConsumer) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    Timer printTimer = new Timer();

    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  velocitySamples.clear();
                  voltageSamples.clear();
                  System.out.println(
                      "********** Advancer " + name + " Console SysId Starting **********");
                }),
            run(this::stopAdvancer).withTimeout(FF_START_DELAY_SECS),
            Commands.runOnce(
                () -> {
                  timer.restart();
                  printTimer.restart();
                }),
            run(
                () -> {
                  double commandedVolts = timer.get() * FF_RAMP_RATE_VOLTS_PER_SEC;
                  voltageConsumer.accept(commandedVolts);

                  double velocityRPM = motor.getVelocityRPM();
                  double appliedVolts = motor.getAppliedVolts();
                  velocitySamples.add(Math.abs(velocityRPM));
                  voltageSamples.add(Math.abs(appliedVolts));

                  if (printTimer.hasElapsed(FF_PRINT_PERIOD_SECS)) {
                    System.out.printf(
                        "Advancer %s SysId: command=%.2f V, applied=%.2f V, velocity=%.2f RPM,"
                            + " position=%.2f rot, current=%.2f A%n",
                        name,
                        commandedVolts,
                        appliedVolts,
                        velocityRPM,
                        motor.getPositionRot(),
                        motor.getSupplyCurrentAmps());
                    printTimer.restart();
                  }
                }))
        .finallyDo(
            () -> {
              stopAdvancer();
              printFeedforwardResults(name, velocitySamples, voltageSamples);
            });
  }

  private static void printFeedforwardResults(
      String name, List<Double> velocitySamples, List<Double> voltageSamples) {
    int n = velocitySamples.size();
    double sumX = 0.0;
    double sumY = 0.0;
    double sumXY = 0.0;
    double sumX2 = 0.0;

    for (int i = 0; i < n; i++) {
      sumX += velocitySamples.get(i);
      sumY += voltageSamples.get(i);
      sumXY += velocitySamples.get(i) * voltageSamples.get(i);
      sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
    }

    NumberFormat formatter = new DecimalFormat("#0.00000");
    System.out.println("********** Advancer " + name + " Console SysId Results **********");
    System.out.println("\tSamples: " + n);

    double denominator = n * sumX2 - sumX * sumX;
    if (n < 2 || Math.abs(denominator) < 1.0e-9) {
      System.out.println("\tNot enough movement to calculate kS/kV.");
      return;
    }

    double kS = (sumY * sumX2 - sumX * sumXY) / denominator;
    double kV = (n * sumXY - sumX * sumY) / denominator;
    System.out.println("\tkS: " + formatter.format(kS));
    System.out.println("\tkV: " + formatter.format(kV));
  }
}
