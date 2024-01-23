package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CANdleSubsystem extends SubsystemBase {


    public boolean isCone;
   

    public CANdleSubsystem() {
       
    }

    /* Wrappers so we can access the CANdle from the subsystem */

  

    public void setIsCone(boolean isCone){
        this.isCone = isCone;
    }

    public boolean getIsCone() {
        return isCone;
    }
}
