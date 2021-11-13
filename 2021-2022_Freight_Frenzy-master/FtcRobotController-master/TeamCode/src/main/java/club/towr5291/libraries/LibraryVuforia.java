package club.towr5291.libraries;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.HINT;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opengl.models.Geometry;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import androidx.annotation.Nullable;

public class LibraryVuforia {

    private static float mmPerInch = 25.4f;
    private static float halfField = 72 * mmPerInch;
    private static float halfTile = 12 * mmPerInch;
    private static float mmTargetHeight = (6) * mmPerInch;

    /**
     * In order for localization to work, we need to tell the system where each target is on the field, and
     * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
     * Transformation matrices are a central, important concept in the math here involved in localization.
     * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     * for detailed information. Commonly, you'll encounter transformation matrices as instances
     * of the {@link OpenGLMatrix} class.
     * <p>
     * If you are standing in the Red Alliance Station looking towards the center of the field,
     * - The X axis runs from your left to the right. (positive from the center to the right)
     * - The Y axis runs from the Red Alliance Station towards the other side of the field
     * where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     * - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     * <p>
     * Before being transformed, each target image is conceptually located at the origin of the field's
     * coordinate system (the center of the field), facing up.
     */
    public enum VuforiaAssetDefinitions {
        //Vuforia Asset Values for FreightFrenzy
        FreightFrenzyBlueStorage     (0,     "BlueStorage",      "FreightFrenzy", -halfField,    (halfTile * 3),     mmTargetHeight, 90, 0, 90),
        FreightFrenzyBlueAllianceWall(1,     "BlueAllianceWall", "FreightFrenzy", halfTile,      halfField,          mmTargetHeight, 90, 0, 0),
        FreightFrenzyRedStorage      (2,     "RedStorage",       "FreightFrenzy", -halfField,    -(halfTile * 3),    mmTargetHeight, 90, 0, 90),
        FreightFrenzyRedAllianceWall (3,     "RedAllianceWall",  "FreightFrenzy", halfTile,      -halfField,         mmTargetHeight, 90, 0, 180),

        //Vuforia Asset Values for UltimateGoal
        UltimateGoalBlueTowerGoal   (0,     "BlueTowerGoal",    "UltimateGoal", halfField,  (halfField/2),  mmTargetHeight, 90, 0, -90),
        UltimateGoalRedTowerGoal    (1,     "RedTowerGoal",     "UltimateGoal", halfField,  -(halfField/2), mmTargetHeight, 90, 0, -90),
        UltimateGoalRedAlliance     (2,     "RedAlliance",      "UltimateGoal", (0),        -halfField,     mmTargetHeight, 90, 0, 180),
        UltimateGoalBlueAlliance    (3,     "BlueAlliance",     "UltimateGoal", (0),        halfField,      mmTargetHeight, 90, 0, 0),
        UltimateGoalFrontWall       (4,     "FrontWall",        "UltimateGoal", -halfField, (0),            mmTargetHeight, 90, 0, 90),

        //Vuforia Asset Values for UltimateGoal
        SkystoneRedPerimeter1   (5,     "Red Perimeter 1",  "Skystone", (halfField/2),  -halfField,     mmTargetHeight, 90, 0, 90),
        SkystoneRedPerimeter2   (6,     "Red Perimeter 2",  "Skystone", -(halfField/2),  -halfField,    mmTargetHeight, 90, 0, 0),
        SkystoneFrontPerimeter1 (7,     "Front Perimeter 1","Skystone", -halfField,     -(halfField/2), mmTargetHeight, 90, 0, 90),
        SkystoneFrontPerimeter2 (8,     "Front Perimeter 2","Skystone", -halfField,     (halfField/2),  mmTargetHeight, 90, 0, 180),
        SkystoneBluePerimeter1  (9,     "Blue Perimeter 1", "Skystone", -(halfField/2), halfField,      mmTargetHeight, 90, 0, 180),
        SkystoneBluePerimeter2  (10,    "Blue Perimeter 2", "Skystone", (halfField/2),  halfField,      mmTargetHeight, 90, 0, 180),
        SkystoneRearPerimeter1  (11,    "Rear Perimeter 1", "Skystone", halfField,      (halfField/2),  mmTargetHeight, 90, 0, 180),
        SkystoneRearPerimeter2  (12,    "Rear Perimeter 2", "Skystone", halfField,      -(halfField/2), mmTargetHeight, 90, 0, 180),
        ;

        private String assetName;
        private String locationName;
        private int locationNumber;
        private Geometry.Point3 dPoint;
        private Geometry.Point3 rPoint;

        VuforiaAssetDefinitions(int _locationNumber, String _locationName, String assetName, float dx, float dy, float dz, float rx, float ry, float rz) {
            locationNumber = _locationNumber;
            locationName = _locationName;
            //Store the left right up position of image target
            dPoint = new Geometry.Point3(dx, dy, dz);
            //Store the rotational values of image target
            rPoint = new Geometry.Point3(rx, ry, rz);
        }

        public static VuforiaAssetDefinitions[] getFreightFrenzyAssets() {
            return new VuforiaAssetDefinitions[] {
                    FreightFrenzyBlueStorage,
                    FreightFrenzyBlueAllianceWall,
                    FreightFrenzyRedStorage,
                    FreightFrenzyRedAllianceWall
            };
        }
        public static VuforiaAssetDefinitions[] getUltimateGoalAssets() {
            return new VuforiaAssetDefinitions[] {
                    UltimateGoalBlueTowerGoal,
                    UltimateGoalRedTowerGoal,
                    UltimateGoalRedAlliance,
                    UltimateGoalBlueAlliance,
                    UltimateGoalFrontWall
            };
        }
        public static VuforiaAssetDefinitions[] getSkyStoneAssets() {
            return new VuforiaAssetDefinitions[] {
                    SkystoneRearPerimeter1,
                    SkystoneRedPerimeter2,
                    SkystoneFrontPerimeter1,
                    SkystoneFrontPerimeter2,
                    SkystoneBluePerimeter1,
                    SkystoneBluePerimeter2,
                    SkystoneRearPerimeter1,
                    SkystoneRearPerimeter2,
            };
        }

        public String getAssetName() {
            return assetName;
        }
        public String getLocationName() {
            return locationName;
        }
        public int getLocationNumber() {
            return locationNumber;
        }
        public Geometry.Point3 getDirectionValues() {
            return dPoint;
        }
        public Geometry.Point3 getRotationalValues() {
            return rPoint;
        }
    }

    private VuforiaLocalizer vuforia;
    private VuforiaTrackables mTrackableTargets;

    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    public LibraryVuforia(HardwareMap hardwareMap, @Nullable WebcamName robotWebcam, boolean ShowView, VuforiaAssetDefinitions[] vuforiaAssetValues) {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters;
        if (ShowView) {
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AVnlHKP/////AAABmU5kpwBUw0KGkmPKLAjP2fthurq7h6D9ULkmkt8zlkqRmNOdUk3BsiLm+o93UF/GumwUEhMEUD2R5SCnKb2GeULtLWeSCLjIRYEGSAfOAnt4vVHboCAvwrOlUykc1WESQrw2sbO+jhb/rw6RVR8v3416VgUUO0AHKPN1M47o0PZO17pIYXVcUYByKSc7fqmm/Lld/XdYbCNBwRJnTFgautU/GsLx193RQSN4GAAtW4yOIyLRC8Ezy6zRIqm2RQdxFh9puI0cB/tDc0oZVtSPBg69MEVmEpP0HloGstMtIgFLpp56eH4rmO/ngHsmIVZ0XbSSvAp68QLdKc6IYYJYQTthftLcP6N4z/avQfOwp1wU";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        if(robotWebcam != null) {
            parameters.cameraName = robotWebcam;
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        }

        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);                                          //enables RGB565 format for the image
        vuforia.enableConvertFrameToBitmap();
        vuforia.setFrameQueueCapacity(5);                                                           //tells VuforiaLocalizer to only store one frame at a time
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        mTrackableTargets = this.vuforia.loadTrackablesFromAsset(vuforiaAssetValues[0].getAssetName());

        //Define where each of the targets are
        for (VuforiaAssetDefinitions definition : vuforiaAssetValues) {
            VuforiaTrackable tmpTrackable = mTrackableTargets.get(definition.getLocationNumber());
            Geometry.Point3 directionalValues = definition.getDirectionValues();
            Geometry.Point3 rotationValues = definition.getRotationalValues();

            tmpTrackable.setName(definition.getLocationName());

            tmpTrackable.setLocation(OpenGLMatrix.translation(directionalValues.x, directionalValues.y, directionalValues.z)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rotationValues.x, rotationValues.y, rotationValues.z)));
        }


        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        // We need to rotate the camera around it's long axis to bring the correct camera forward.
                        CAMERA_CHOICE == BACK ? -90 : 90,
                        0,
                        // Rotate the phone vertical about the X axis if it's in portrait mode
                        PHONE_IS_PORTRAIT ? 90 : 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : mTrackableTargets) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    public LibraryVuforia(HardwareMap hardwareMap, boolean ShowView, VuforiaAssetDefinitions[] vuforiaAssetValues) {
        this(hardwareMap, null, ShowView, vuforiaAssetValues);
    }

    public VuforiaTrackable getTrackable(VuforiaAssetDefinitions trackable) {
        return mTrackableTargets.get(trackable.getLocationNumber());
    }

    public VuforiaTrackables getTrackables() {
        return this.mTrackableTargets;
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }
}