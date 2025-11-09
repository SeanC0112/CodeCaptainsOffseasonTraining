package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command IntakeCmd() {
    return run(() -> runRollers(rollersSpeedIn.get())).finallyDo(() -> runRollers(0));
  }

  public Command EjectCmd() {
    return run(() -> runRollers(rollersSpeedOut.get())).finallyDo(() -> runRollers(0));
  }

  private void runFrontRollers(double volts) {
    io.runFrontRollers(volts);
  }

  private void runBackRollers(double volts) {
    io.runBackRollers(volts);
  }

  private void runRollers(double volts) {
    io.runRollers(volts);
  }
}
