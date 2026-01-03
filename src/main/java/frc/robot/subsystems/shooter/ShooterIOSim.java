package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim flywheelTop =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), jKgMetersSquaredTop, 1), DCMotor.getKrakenX60(1));
    private final FlywheelSim flywheelBottom =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), jKgMetersSquaredBottom, 1), DCMotor.getKrakenX60(1));

    private final SingleJointedArmSim armSim =
        new SingleJointedArmSim(DCMotor.getKrakenX60(2), gearRatio, jKgMetersSquaredArm, armLengthMeters, 0, maxAngleRads, true, 0);

    private double armAppliedVolts;

    public ShooterIOSim() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelTopAppliedVolts = flywheelTop.getInputVoltage();
        inputs.flywheelBottomAppliedVolts = flywheelBottom.getInputVoltage();

        inputs.flywheelTopCurrentAmps = flywheelTop.getCurrentDrawAmps();
        inputs.flywheelBottomCurrentAmps = flywheelBottom.getCurrentDrawAmps();

        inputs.flywheelTopVelocityRPM = flywheelTop.getAngularVelocityRPM();
        inputs.flywheelBottomVelocityRPM = flywheelBottom.getAngularVelocityRPM();

        inputs.armLeftAppliedVolts = armAppliedVolts;
        inputs.armRightAppliedVolts = armAppliedVolts;

        inputs.armLeftCurrentAmps = armSim.getCurrentDrawAmps();
        inputs.armRightCurrentAmps = armSim.getCurrentDrawAmps();

        inputs.armLeftAngleDegs = Units.radiansToDegrees(armSim.getAngleRads());
        inputs.armRightAngleDegs = Units.radiansToDegrees(armSim.getAngleRads());
    }

    @Override
    public void runFlywheels(double volts) {
        flywheelTop.setInputVoltage(volts);
        flywheelBottom.setInputVoltage(volts);
    }

    @Override
    public void runFlywheelTop(double volts) {
        flywheelTop.setInputVoltage(volts);
    }

    @Override
    public void runFlywheelBottom(double volts) {
        flywheelBottom.setInputVoltage(volts);
    }

    @Override
    public void stopFlywheelTop() {
        flywheelTop.setInputVoltage(0);
    }

    @Override
    public void stopFlywheelBottom() {
        flywheelBottom.setInputVoltage(0);
    }

    @Override
    public void runArmMotors(double volts) {
        armAppliedVolts = volts;
        armSim.setInputVoltage(armAppliedVolts);
    }

    @Override
    public void stopArmMotors() {
        armAppliedVolts = 0;
        armSim.setInputVoltage(armAppliedVolts);
    }

    @Override
    public void setArmEncoders(double angleDegs) {
        armSim.setState(Units.degreesToRadians(angleDegs), 0);
    }
}
