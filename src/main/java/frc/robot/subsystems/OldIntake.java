 package frc.robot.subsystems;

 import com.ctre.phoenix6.hardware.TalonFX;
 import com.ctre.phoenix6.signals.NeutralModeValue;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

 import com.ctre.phoenix6.configs.TalonFXConfiguration;


 public final class OldIntake extends SubsystemBase {
     // values adjusted dont change w out intention
     private final double intake_speed = 0.2;
     private final double outtake_speed = 0.1; //.7025, .69, .6, 0.5, 0.1 works muy well
     private final double full_speed = 0.15;
     private final double hola = 0.05;
     private final double adios = 0.2;
     private final double centre = 0.15;

     public TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();

     private final TalonFX intakeMotor1 = new TalonFX(10);
     private final TalonFX intakeMotor2 = new TalonFX(11);

     public OldIntake() {

         /* Clear Sticky Faults */
         intakeMotor1.clearStickyFaults();
         intakeMotor2.clearStickyFaults();

         /* Open Loop Ramping */
         intakeConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
         intakeConfiguration.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

         /* Closed Loop Ramping */
         intakeConfiguration.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
         intakeConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

         /* Peak Output */
         intakeConfiguration.MotorOutput.PeakForwardDutyCycle = 1;
         intakeConfiguration.MotorOutput.PeakReverseDutyCycle = -1;

         /* Neutral Mode */
         intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

         /* Current Limiting */
         intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
         intakeConfiguration.CurrentLimits.SupplyCurrentLimit = 30;
         intakeConfiguration.CurrentLimits.SupplyCurrentThreshold = 50;
         intakeConfiguration.CurrentLimits.SupplyTimeThreshold = 0.08;

         /* Voltage */
         intakeConfiguration.Voltage.PeakForwardVoltage = 13; //may need to remove, could be equivalent to configVoltageCompSat. same as ln 42
         intakeConfiguration.Voltage.PeakReverseVoltage = -12;
         intakeConfiguration.Voltage.SupplyVoltageTimeConstant = 0.1;

        //  pheonix 6 version of setstatusframeperiod
         intakeMotor1.getPosition().setUpdateFrequency(252);
         intakeMotor2.getPosition().setUpdateFrequency(252);

           /* Factory Defaults/Applies Config */
         intakeMotor1.getConfigurator().apply(intakeConfiguration);
         intakeMotor2.getConfigurator().apply(intakeConfiguration);
     }

    //   intake methods
    //  unclear whether intakle parameter units are percentage or degrees
     public void intake() {  //m_1 is postive m_2 is negative, outake vis versa
         intakeMotor1.set(intake_speed);
         intakeMotor2.set(-intake_speed);
     }

     public void outtake() {
        intakeMotor1.set(-outtake_speed);
        intakeMotor2.set(outtake_speed);
        // intakeMotor1.set(-outtake_speed);
     }

     public void hold() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);

     }

     public void stopIntake() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);
     }

    public void fullOut() {
        intakeMotor1.set(-full_speed);
        intakeMotor2.set(full_speed);
    }

    public void hola() {
        intakeMotor1.set(-hola);
        intakeMotor2.set(hola);
    }

    public void adios() {
         intakeMotor1.set(-adios);
        intakeMotor2.set(adios);
    }

    public void centre() {
        intakeMotor1.set(-centre);
        intakeMotor2.set(centre);
    }


     }
     
 
    





