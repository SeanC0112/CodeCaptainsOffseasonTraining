package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import edu.wpi.first.math.util.Units;


import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterIOTalonFX implements ShooterIO {
    private TalonFX flywheelTop = new TalonFX(2);
    private TalonFX flywheelBottom = new TalonFX(3);

    private TalonFX armLeft = new TalonFX(4);
    private TalonFX armRight = new TalonFX(5);

    public ShooterIOTalonFX() {}

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelTopAppliedVolts = flywheelTop.getSupplyVoltage().getValueAsDouble();
        inputs.flywheelBottomAppliedVolts = flywheelBottom.getSupplyVoltage().getValueAsDouble();

        inputs.flywheelTopCurrentAmps = flywheelTop.getSupplyCurrent().getValueAsDouble();
        inputs.flywheelBottomCurrentAmps = flywheelBottom.getSupplyCurrent().getValueAsDouble();

        inputs.flywheelTopVelocityRPM = flywheelTop.getRotorVelocity().getValueAsDouble();
        inputs.flywheelBottomVelocityRPM = flywheelBottom.getRotorVelocity().getValueAsDouble();

        inputs.armLeftAppliedVolts = armLeft.getSupplyVoltage().getValueAsDouble();
        inputs.armRightAppliedVolts = armRight.getSupplyVoltage().getValueAsDouble();

        inputs.armLeftCurrentAmps = armLeft.getSupplyCurrent().getValueAsDouble();
        inputs.armRightCurrentAmps = armRight.getSupplyCurrent().getValueAsDouble();

        inputs.armLeftAngleDegs = Units.rotationsToDegrees(armLeft.getRotorPosition().getValueAsDouble())/gearRatio;
        inputs.armRightAngleDegs = Units.rotationsToDegrees(armRight.getPosition().getValueAsDouble())/gearRatio;
    }
}
