package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeIOSim implements IntakeIO {

  private double rollersFrontAppliedVolts = 0.0;
  private double rollersBackAppliedVolts = 0.0;

  public IntakeIOSim() {}

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollersFrontAppliedVolts = rollersFrontAppliedVolts;
    inputs.rollersBackAppliedVolts = rollersBackAppliedVolts;
  }

  @Override
  public void runFrontRollers(double volts) {
    rollersFrontAppliedVolts = volts;
  }

  @Override
  public void runBackRollers(double volts) {
    rollersBackAppliedVolts = volts;
  }

  @Override
  public void runRollers(double volts) {
    rollersBackAppliedVolts = volts;
    rollersFrontAppliedVolts = volts;
  }
}
