package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

// import java.awt.geom.Line2D;


public final class Constants {
    public static final double stickDeadband = 0.2;

    public static final class Swerve {

         public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4); //used to be 0.47
         public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
         public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
         public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);


        
        public static final int pigeonID = 0;
        // public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.MK4i.KrakenX60(COTSFalconSwerveConstants.MK4i.driveRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(25); 
        public static final double wheelBase = Units.inchesToMeters(25); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final  SensorDirectionValue cancoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35; //20
        public static final int angleCurrentThreshold = 55;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35; //35
        public static final int driveCurrentThreshold = 55; //60
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        
        


        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;



        /* Drive Motor PID Values */
        public static final double driveKP = 0.5; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        
        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.32; //.32, .55, .65, .95
        public static final double driveKV = 1.51; //1.51, 3.51
        public static final double driveKA = 0.27; 
        

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 2.0;  //6.0 
         /** Radians per Second */
        public static final double maxAngularVelocity = 2.5;  //3.0


        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
     
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft { 
            public static final int driveMotorID = 18; //d5
            public static final int angleMotorID = 15; //t4
            public static final int canCoderID = 3; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-9.580078125);//65.83, 61.76, 107.9296 106.2595
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        /* Front Right Module - Module 1 */
        public static final class FrontRight { 
            public static final int driveMotorID = 19; //d1 --> d8
            public static final int angleMotorID = 0; //t8 --> t1
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-48.4277444);//265.34//-14.94, 341.91, 81.6504
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class RearLeft { 
            public static final int driveMotorID = 16; //d3
            public static final int angleMotorID = 20; //t6
            public static final int canCoderID = 1; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(19.160156);//2.19//59.77//180, 59.62
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class RearRight { 
            public static final int driveMotorID = 17; 
            public static final int angleMotorID = 14; 
            public static final int canCoderID = 2; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-11.25);//166.20//183.03//-1.32, 180 -8.4375 0.087890625
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

         public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(0.5, 0, 0), // Translation PID
                new PIDConstants(10, 0, 0), // Rotation PID 25, 7, 3.5 doesn't work, og 10 0 0
                maxSpeed,
                flModuleOffset.getNorm(),
                 new ReplanningConfig()
        );

        

    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 4.5; //1.5 orignally
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //2 originally
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class LimelightConstants {
        public static final double m_limelightMountAngleDegree = 50.25;
        public static final double m_limelightLensHeightInches = 10;
        public static final double m_limelightToFrontOfRobot = 0;
      }

    public interface PhotonVision {

        String APRIL_TAGS_FRONT_RIGHT_CAMERA_NAME = "AprilTags_Front_Right";
        String APRIL_TAGS_REAR_LEFT_CAMERA_NAME = "AprilTags_Rear_Left";

        String APRIL_TAGS_PIPELINE_NAME = "AprilTags";
        String NOTES_PIPELINE_NAME = "IdentifyNotes";

        Double NOTES_INDEXER_CAMERA_ANGLE = 90d;

         Transform3d ROBOT_TO_TAG_FRONT_RIGHT_CAM_POS =
                new Transform3d(new Translation3d(Units.inchesToMeters(11.5d), Units.inchesToMeters(-10.7d), Units.inchesToMeters(10.0d)), new Rotation3d(0d, Math.toRadians(-38d), 0/*Math.PI+3.125*/));
        Transform3d ROBOT_TO_TAG_REAR_LEFT_CAM_POS =
                new Transform3d(new Translation3d(Units.inchesToMeters(-11.5d), Units.inchesToMeters(10.7d), Units.inchesToMeters(10.0d)), new Rotation3d(0d, Math.toRadians(-38d), Math.PI/*3.125*/));

        AprilTagFieldLayout tagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4d, 4d, 8d);
        Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5d, 0.5d, 1d);
    }
   
    

}
