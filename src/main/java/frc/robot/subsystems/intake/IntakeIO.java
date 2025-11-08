package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollersFrontAppliedVolts = 0.0;
    public double rollersFrontCurrentAmps = 0.0;

    public double rollersBackAppliedVolts = 0.0;
    public double rollersBackCurrentAmps = 0.0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  default void runFrontRollers(double volts) {}

  default void runBackRollers(double volts) {}

  default void runRollers(double volts) {}
}
