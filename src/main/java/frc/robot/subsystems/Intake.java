 package frc.robot.subsystems;

 import com.ctre.phoenix6.hardware.TalonFX;
 import com.ctre.phoenix6.signals.NeutralModeValue;
 import edu.wpi.first.wpilibj2.command.SubsystemBase;

 import com.ctre.phoenix6.configs.TalonFXConfiguration;


 public final class Intake extends SubsystemBase {
     // value grabbed from previous season
     private final double intake_speed = 0.3;
     public TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration();

     private final TalonFX intakeMotor1 = new TalonFX(10);
     private final TalonFX intakeMotor2 = new TalonFX(11);

     public Intake() {

         /* Clear Sticky Faults */
         intakeMotor1.clearStickyFaults();
         intakeMotor2.clearStickyFaults();

         /* Open Loop Ramping */
         intakeConfiguration.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;

         /* Peak Output */
         intakeConfiguration.MotorOutput.PeakForwardDutyCycle = 1;
         intakeConfiguration.MotorOutput.PeakReverseDutyCycle = -1;

         /* Neutral Mode */
         intakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

         /* Current Limiting */
         intakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
         intakeConfiguration.CurrentLimits.SupplyCurrentLimit = 20;
         intakeConfiguration.CurrentLimits.SupplyCurrentThreshold = 40;
         intakeConfiguration.CurrentLimits.SupplyTimeThreshold = 0.08;

        //  pheonix 6 version of setstatusframeperiod
         intakeMotor1.getPosition().setUpdateFrequency(252);

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
        //   intakeMotor1.set(-intake_speed);
         intakeMotor2.set(-0.35);
     }

     public void hold() {
         intakeMotor1.set(0);
         intakeMotor2.set(0);

     }

     public void stopIntake() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);

     }
     
 }
    





