package frc.robot.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSource extends SubsystemBase {

   private double ta;
   private double tx;
   private double ty;

   NetworkTableEntry prelimtx;
   NetworkTableEntry prelimty;
   NetworkTableEntry prelimta;
   NetworkTableEntry prelimCamtran;
   NetworkTable table;
   NetworkTableInstance Inst;

   public LimelightSource() {
      Inst = NetworkTableInstance.getDefault();
      table = Inst.getTable("limelight-intake");
      prelimta = table.getEntry("ta");
      prelimtx = table.getEntry("tx");
      prelimty = table.getEntry("ty");
      
   }

   public void updateGameState(){
      ta = prelimta.getDouble(0);
      tx = prelimtx.getDouble(0);
      ty = prelimty.getDouble(0);
   }

   public double getArea(){
      ta = prelimta.getDouble(0);
      return ta;
   }

   public double getX(){
      tx = prelimtx.getDouble(0);
      return tx;
   }

   public double getY(){
      ty = prelimty.getDouble(0);
      return ty;
   }

   public void visionMode(){
      NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("ledMode").setNumber(3);
      NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("camMode").setNumber(3); //was 0
   }

   public void cameraMode(){
      NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("ledMode").setNumber(1);
      NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("camMode").setNumber(1);
   }

   public void updateDashboard() {
	   SmartDashboard.putNumber("source ta", getArea());
      SmartDashboard.putNumber("source tx", getX());
      SmartDashboard.putNumber("source ty", getY());
	}
}