package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double flywheelTopAppliedVolts;
        public double flywheelTopCurrentAmps;
        public double flywheelBottomAppliedVolts;
        public double flywheelBottomCurrentAmps;

        public double flywheelTopVelocityRPM;
        public double flywheelBottomVelocityRPM;

        public double armLeftAppliedVolts;
        public double armLeftCurrentAmps;
        public double armRightAppliedVolts;
        public double armRightCurrentAmps;

        public double armLeftAngleDegs;
        public double armRightAngleDegs;
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    default void runFlywheels(double volts) {}

    default void runFlywheelTop(double volts) {}

    default void runFlywheelBottom(double volts) {}

    default void stopFlywheels() {}

    default void stopFlywheelTop() {}

    default void stopFlywheelBottom() {}

    default void runArmMotors(double volts) {}

    default void stopArmMotors() {}

    default void setArmEncoders(double angleDegs) {}
}
