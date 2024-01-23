package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final TalonSRX intakeMotor = new TalonSRX(11);


    public Intake() {
        intakeMotor.configFactoryDefault();
        intakeMotor.clearStickyFaults();
        intakeMotor.configOpenloopRamp(0.25);
        intakeMotor.configPeakOutputForward(1);
        intakeMotor.configPeakOutputReverse(-1);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 50, 0.2));
        intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 252, 10);

    }

    // Cone Commands
    public void intakeCone() {
        intakeMotor.set(ControlMode.PercentOutput, -0.2); //old = .6
    }

    public void holdCone(){
        intakeMotor.set(ControlMode.PercentOutput, 0.0); //set to .1 or 0.05
    }

    public void scoreCone() {
        intakeMotor.set(ControlMode.PercentOutput, 0.35);
    }

    // Cube Commands
    public void intakeCube() {
        intakeMotor.set(ControlMode.PercentOutput, 0.4);
    }

    public void holdCube() {
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    public void scoreCube(){
        intakeMotor.set(ControlMode.PercentOutput, -0.4);
    }

    public void spitCube(){
        intakeMotor.set(ControlMode.PercentOutput, -0.3);
    }

    // Stop Intake
    public void stopIntake(){
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    // Get Power Draw
    public double getIntakeAmps(){
        return intakeMotor.getSupplyCurrent();
    }

}
