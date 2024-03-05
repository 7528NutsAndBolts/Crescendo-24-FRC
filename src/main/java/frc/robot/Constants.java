package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
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
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

// import java.awt.geom.Line2D;


public final class Constants {
    public static final double stickDeadband = 0.2;

    public static final class Swerve {
        public static final int pigeonID = 0;
        // public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.MK4i.KrakenX60(COTSFalconSwerveConstants.MK4i.driveRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24); 
        public static final double wheelBase = Units.inchesToMeters(24); 
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
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
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
        public static final double driveKP = 6.0; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        
        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = 0.32 / 12; //TODO used to be divded by 12
        public static final double driveKV = 1.51 / 12;
        public static final double driveKA = 0.27 / 12;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 1.0;        /** Radians per Second */
        public static final double maxAngularVelocity = 2.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
     
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeft { 
            public static final int driveMotorID = 5; //d5
            public static final int angleMotorID = 4; //t4
            public static final int canCoderID = 1; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(49.4824);//65.83, 61.76
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRight { 
            public static final int driveMotorID = 1; //d1 --> d8
            public static final int angleMotorID = 8; //t8 --> t1
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-51.9434);//265.34//-14.94, 341.91
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class RearLeft { 
            public static final int driveMotorID = 3; //d3
            public static final int angleMotorID = 6; //t6
            public static final int canCoderID = 0; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(262.5293);//2.19//59.77//180, 59.62
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class RearRight { 
            public static final int driveMotorID = 2; //d7
            public static final int angleMotorID = 7; //t2
            public static final int canCoderID = 2; 
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(225);//166.20//183.03//-1.32, 180
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3; //1.5 orignally
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
    


     interface PathPlannerConstants {

        PathConstraints pathConstraints =
                new PathConstraints(4, 3,
                        2 * Math.PI, 1.5 * Math.PI);

        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(6.0, 0, 0), // Translation PID
                new PIDConstants(25.0, 7.0, 1.5), // Rotation PID
                4, // Max Speed Meters per second
                Math.sqrt(Swerve.trackWidth * Swerve.trackWidth +
                        Swerve.wheelBase * Swerve.wheelBase) / 2, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()
        );
    }

//     public interface GameStateConstants {
//         enum RobotCycleStatus {
//             NO_NOTE_CANT_SEE_SOURCE,
//             NO_NOTE_SEES_SPEAKER,
//             HAS_NOTE_SEES_SOURCE,
            
//             NO_NOTE_CANT_SEE_AMP,
//             NO_NOTE_SEES_AMP,
//             HAS_NOT_CANT_SEE_AMP,
//             HAS_NOTE_SEES_AMP
//         }

//         enum INTAKEMODE {
//             SOURCE,
//             AMP
//         }
//     }

//     public interface FieldCoordinates {
//         // Blue Alliance stage post coordinates:
//         Translation2d BLUE_STAGE_AMP_POINT = new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(227.12));
//         Translation2d BLUE_STAGE_SPEAKER_POINT = new Translation2d(Units.inchesToMeters(116.5), Units.inchesToMeters(161.62));
//         Translation2d BLUE_STAGE_SOURCE_POINT = new Translation2d(Units.inchesToMeters(230), Units.inchesToMeters(96.12));

//         // Red Alliance stage post coordinates
//         Translation2d RED_STAGE_AMP_POINT = new Translation2d(Units.inchesToMeters(412.8), Units.inchesToMeters(227.12));
//         Translation2d RED_STAGE_SPEAKER_POINT = new Translation2d(Units.inchesToMeters(526.25), Units.inchesToMeters(161.62));
//         Translation2d RED_STAGE_SOURCE_POINT = new Translation2d(Units.inchesToMeters(412.8), Units.inchesToMeters(96.12));

//         // Blue Alliance Stage sides
//         Line2D BLUE_STAGE_LEFT = new Line2D.Double(BLUE_STAGE_SPEAKER_POINT.getX(), BLUE_STAGE_SPEAKER_POINT.getY(), BLUE_STAGE_AMP_POINT.getX(), BLUE_STAGE_AMP_POINT.getY());
//         Line2D BLUE_CENTER_STAGE = new Line2D.Double(BLUE_STAGE_SOURCE_POINT.getX(), BLUE_STAGE_SOURCE_POINT.getY(), BLUE_STAGE_AMP_POINT.getX(), BLUE_STAGE_AMP_POINT.getY());
//         Line2D BLUE_STAGE_RIGHT = new Line2D.Double(BLUE_STAGE_SPEAKER_POINT.getX(), BLUE_STAGE_SPEAKER_POINT.getY(), BLUE_STAGE_SOURCE_POINT.getX(), BLUE_STAGE_SOURCE_POINT.getY());

//         // Red Alliance Stage sides
//         Line2D RED_STAGE_RIGHT = new Line2D.Double(RED_STAGE_SPEAKER_POINT.getX(), RED_STAGE_SPEAKER_POINT.getY(), RED_STAGE_AMP_POINT.getX(), RED_STAGE_AMP_POINT.getY());
//         Line2D RED_CENTER_STAGE = new Line2D.Double(RED_STAGE_AMP_POINT.getX(), RED_STAGE_AMP_POINT.getY(), RED_STAGE_SOURCE_POINT.getX(), RED_STAGE_SOURCE_POINT.getY());
//         Line2D RED_STAGE_LEFT = new Line2D.Double(RED_STAGE_SPEAKER_POINT.getX(), RED_STAGE_SPEAKER_POINT.getY(), RED_STAGE_AMP_POINT.getX(), RED_STAGE_AMP_POINT.getY());

//         // Speaker coordinates
//         Translation3d BLUE_SPEAKER = new Translation3d(
//                 Units.inchesToMeters(-1.5),
//                 Units.inchesToMeters(218.42),
//                 Units.inchesToMeters(80));
//         Translation3d RED_SPEAKER = new Translation3d(
//                 Units.inchesToMeters(652.73),
//                 Units.inchesToMeters(218.42),
//                 Units.inchesToMeters(80)
//         );
//     }

//     double FIELD_AMP_OPENING_WIDTH = 0.6096;
//     double FIELD_AMP_OPENING_HEIGHT = 0.4572;
//     double FIELD_AMP_OPENING_HEIGHT_FROM_FLOOR = 0.6604;
//     double FIELD_AMP_APRIL_TAG_HEIGHT_TO_CENTER = 1.3557;

//     double FIELD_SPEAKER_OPENING_HEIGHT_FROM_FLOOR = 1.9812;
//     double FIELD_SPEAKER_OPENING_WIDTH = 1.0509;
//     //double FIELD_SPEAKER_OPENING_HEIGHT_AT_CENTER = ;
//     double FIELD_SPEAKER_OPENING_OVERHANG = 0.4572;
//     double FIELD_SPEAKER_APRIL_TAG_HEIGHT_OF_BOTTOM = 1.3574;
//     double FIELD_SPEAKER_APRIL_TAG_OFFSET_DISTANCE = 0.5668;

//     double FIELD_SOURCE_APRIL_TAG_HEIGHT_BOTTOM = 1.2224;
//     double FIELD_SOURCE_APRIL_TAG_DISTANCE_BETWEEN = 0.98425 + 0.2667;




//     // public static final class VisionConstants {
//     //     public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(0);
//     //     public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18);
//     //     public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
//     //   }

//     // public static final class StartingConstants {
//     //     public static final Pose2d aprilTag1 = new Pose2d(13.313558, 1.071626, new Rotation2d(Math.toRadians(180)));
//     //     public static final Pose2d aprilTag2 = new Pose2d(13.313558, 2.748026, new Rotation2d(Math.toRadians(180)));
//     //     public static final Pose2d aprilTag3 = new Pose2d(13.313558, 4.424426, new Rotation2d(Math.toRadians(180)));
//     //     public static final Pose2d aprilTag6 = new Pose2d(2.2, 4.424426, new Rotation2d(Math.toRadians(180)));
//     //     public static final Pose2d aprilTag7 = new Pose2d(2.2, 2.748026, new Rotation2d(Math.toRadians(180)));
//     //     public static final Pose2d aprilTag8 = new Pose2d(2.2, 1.071626, new Rotation2d(Math.toRadians(180)));

//     //     public static final Pose2d[] startingPositions = new Pose2d[]{
//     //         aprilTag1,
//     //         aprilTag2,
//     //         aprilTag3,
//     //         null,
//     //         null,
//     //         aprilTag6,
//     //         aprilTag7,
//     //         aprilTag8
//     //     };
        
    

//     // public static final class PneumaticsConstants { 
//     //     public static final int PneumaticsHubID = 20;
//     //     public static final int ClawSolenoidID = 0;
//     // }
    
//     // public static final class Clawstants { 
//     //     public static final int ClawMotorWristID = 28;
//     //     public static final int ClawMotorLeftID = 26;
//     //     public static final int ClawMotorRightID = 25;
//     //     // public static final int PotentiometerID = 0;
        
//     //     public static final double armFeedForward = 0.03;
//     //     public static final double wristFeedForward = 0.03;

//     //     public static final double ClawLoadingToLowPosition = 30000; // thirty-thousand

//     //     public static final double armGearRatio = 144;
//     //     public static final double wristGearRatio = 30;

//     //     /* Arm Motor PID Values */
//     // }
// }

}
