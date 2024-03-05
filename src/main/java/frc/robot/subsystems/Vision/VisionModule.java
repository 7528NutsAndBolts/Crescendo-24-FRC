// package frc.robot.subsystems.Vision;


// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import frc.robot.Constants;
// import frc.robot.Constants.Swerve;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;

// import java.util.Optional;

// public final class VisionModule implements Constants.PhotonVision {

//     private final PhotonCamera aprilTagsFrontRight;
//     private final PhotonCamera aprilTagsRearLeft;
//     private PhotonPoseEstimator photonEstimatorFrontRight;
//     private PhotonPoseEstimator photonEstimatorRearLeft;

//     private PhotonPipelineResult lastNoteResult = null;
//     private boolean startTrackingNotes = false;


//     private static VisionModule instance;

//     private VisionModule() {
//         PhotonCamera.setVersionCheckEnabled(false);

//         aprilTagsFrontRight = new PhotonCamera(APRIL_TAGS_FRONT_RIGHT_CAMERA_NAME);
//         aprilTagsRearLeft = new PhotonCamera(APRIL_TAGS_REAR_LEFT_CAMERA_NAME);

//         photonEstimatorFrontRight =
//                 new PhotonPoseEstimator(
//                         tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsFrontRight, ROBOT_TO_TAG_FRONT_RIGHT_CAM_POS);
//         photonEstimatorFrontRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

//         photonEstimatorRearLeft =
//                 new PhotonPoseEstimator(
//                         tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsRearLeft, ROBOT_TO_TAG_REAR_LEFT_CAM_POS);
//         photonEstimatorRearLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


//     }

//     public static synchronized VisionModule getInstance() {
//         if (instance == null) {
//             instance = new VisionModule();
//         }
//         return instance;
//     }

//     public PhotonCamera getAprilTagsFrontRightCamera() {
//         return aprilTagsFrontRight;
//     }

//     public void setPhotonEstimatorFrontRight(Transform3d transform3d) {
//         this.photonEstimatorFrontRight = new PhotonPoseEstimator(
//                 tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsFrontRight, transform3d);
//         photonEstimatorFrontRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     public void setPhotonEstimatorRearLeft(Transform3d transform3d) {
//         this.photonEstimatorRearLeft = new PhotonPoseEstimator(
//                 tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, aprilTagsRearLeft, transform3d);
//         photonEstimatorFrontRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//     }

//     public PhotonCamera getAprilTagsRearLeftCamera() {
//         return aprilTagsRearLeft;
//     }

//     public PhotonPoseEstimator getPhotonEstimatorFrontRight() {
//         return photonEstimatorFrontRight;
//     }

//     public PhotonPoseEstimator getPhotonEstimatorRearLeft() {
//         return photonEstimatorRearLeft;
//     }


//     public PhotonPipelineResult getLastNoteResult() {
//         return lastNoteResult;
//     }

//     protected void setLastNoteResult(PhotonPipelineResult lastNoteResult) {
//         this.lastNoteResult = lastNoteResult;
//     }

//     public boolean isStartTrackingNotes() {
//         return startTrackingNotes;
//     }

//     public void setStartTrackingNotes(boolean startTrackingNotes) {
//         this.startTrackingNotes = startTrackingNotes;
//     }

//     public void trackPipelineResults() {
//     }

// }