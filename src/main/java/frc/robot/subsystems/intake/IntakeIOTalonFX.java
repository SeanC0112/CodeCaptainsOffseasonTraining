package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rollersFront = new TalonFX(0); //TODO add device ID
    private final TalonFX rollersBack = new TalonFX(1);

    public IntakeIOTalonFX(IntakeIOInputs inputs) {
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollersFrontAppliedVolts = rollersFront.getMotorVoltage().getValueAsDouble();
        inputs.rollersFrontCurrentAmps = rollersFront.getSupplyCurrent().getValueAsDouble();

        inputs.rollersBackAppliedVolts = rollersBack.getMotorVoltage().getValueAsDouble();
        inputs.rollersBackCurrentAmps = rollersBack.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void runFrontRollers(double volts) {
        rollersFront.setControl(new VoltageOut(volts));
    }

    @Override
    public void runBackRollers(double volts) {
        rollersBack.setControl(new VoltageOut(volts));
    }
    
    @Override
    public void runRollers(double volts) {
        rollersFront.setControl(new VoltageOut(volts));
        rollersBack.setControl(new VoltageOut(volts));
    }
}