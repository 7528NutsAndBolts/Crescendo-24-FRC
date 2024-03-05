// package frc.lib.util;


// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.Constants;


// import static frc.robot.Constants.FieldCoordinates.BLUE_STAGE_AMP_POINT;

// public class FieldState implements Constants.GameStateConstants {

//     private static FieldState instance;

//     private boolean[] centerNotesExist;
//     private int centerNoteIndex;
//     private boolean centerNotesGone;
//     private boolean onRedAlliance;

//     private Translation2d ampCoords = BLUE_STAGE_AMP_POINT;

//     private FieldState() {
//         centerNotesExist = new boolean[]{
//                 true,
//                 true,
//                 true,
//                 true,
//                 true
//         };

//         centerNotesGone = false;
//         centerNoteIndex = 0;

//         var alliance = DriverStation.getAlliance();
//         onRedAlliance = alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
//     }

//     public static synchronized FieldState getInstance() {
//         if (instance == null) {
//             instance = new FieldState();
//         }
//         return instance;
//     }

//     public boolean[] getCenterNotesExist() {
//         return centerNotesExist;
//     }

//     public void setCenterNotesExist(boolean[] centerNotesExist) {
//         this.centerNotesExist = centerNotesExist;
//     }

//     public void setCenterNoteExists(int index, boolean val) {
//         if (index < centerNotesExist.length) centerNotesExist[index] = val;
//     }

//     public void setCenterNoteIndex(int centerNoteIndex) {
//         this.centerNoteIndex = centerNoteIndex;
//     }

//     public int getCenterNoteIndex() {
//         return centerNoteIndex;
//     }

//     public void setCenterNotesGone(boolean centerNotesGone) {
//         this.centerNotesGone = centerNotesGone;
//     }

//     public boolean isCenterNotesGone() {
//         return centerNotesGone;
//     }

//     public boolean onRedAlliance() {
//         return onRedAlliance;
//     }

//     public void setOnRedAlliance(boolean onRedAlliance) {
//         this.onRedAlliance = onRedAlliance;
//     }

//     public Translation2d getAmpCoords() {
//         return ampCoords;
//     }

//     public void setAmpCoords(Translation2d ampCoords) {
//         this.ampCoords = ampCoords;
//     }
// }