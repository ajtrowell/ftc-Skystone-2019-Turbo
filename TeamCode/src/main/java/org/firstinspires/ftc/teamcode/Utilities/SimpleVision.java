package org.firstinspires.ftc.teamcode.Utilities;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Reads Vuforia markers off the camera.
 * Requires the Vuforia key is set in SharedCode/src/main/res/values/vuforia.xml.
 * Example:
 *   private SimpleVision vuforia;
 *
 *   public void init() {
 *       String vuforiaKey = "...";
 *       vuforia = new SimpleVision(vuforiaKey);
 *   }
 *
 *   public void loop() {
 *       RelicRecoveryVuMark vuMark = vuforia.detectMark();
 *       if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
 *           // ... use detected mark.
 *       }
 *   }
 *
 * Thanks to Phillip Tischler  http://pmtischler-ftc-app.readthedocs.io/en/latest/
 */
public class SimpleVision {

    // opMode reference
    private RobotHardware opMode;
    private ElapsedTime tfTimer;
    public static final String TAG = "simpleVision class";

    // The external Vuforia ID localizer.
    private VuforiaLocalizer vuforia;
    private boolean useBackCamera;

    // Vuforia Trackables
    private boolean useVuforiaTrackables;
    private VuforiaTrackables targetsSkystone;
    private List<VuforiaTrackable> allTrackables;


    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    // Vuforia Nav Geometry Properties
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    private boolean PHONE_IS_PORTRAIT = true;


    // TensorFlow Properties
    public boolean tfSupported = true;
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/Skystone.tflite"; //For OpenRC, loaded from internal storage to reduce APK size
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private TFObjectDetector tfod;
    public List<Recognition> updatedRecognitions;
    public List<Recognition> pastRecognitions;

    // Webcam Support
    private boolean useWebcam;
    int captureCounter = 0;
    File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    WebcamName webcamName;


    public static enum SkystoneSets {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
    }

    public static enum SkystoneIdentificationLocation {
        CENTER,
        BOTTOM,
    }

    public SkystoneSets skystonePositions = SkystoneSets.UNKNOWN;


    /**
     * Creates a Vuforia localizer and starts localization.
     * @param vuforiaLicenseKey The license key to access Vuforia code.
     */
    public SimpleVision(String vuforiaLicenseKey, RobotHardware opMode, boolean useVuforiaMonitor,
                        boolean useTensorFlowMonitor, boolean useBackCamera, boolean useVuforiaTrackables,
                        boolean useWebcam) {
        this.opMode = opMode;
        this.useBackCamera = useBackCamera;
        this.useWebcam = useWebcam;

        this.useVuforiaTrackables = useVuforiaTrackables;
        VuforiaLocalizer.Parameters parameters;
        if (useVuforiaMonitor) {
            // Show camera
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else { // Don't use camera monitor
            parameters = new VuforiaLocalizer.Parameters();
        }
        parameters.vuforiaLicenseKey = vuforiaLicenseKey;
         if (useWebcam){ // useWebcam
             try {
                 webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
                 parameters.cameraName = webcamName;
                 // Might be required for webcam (Phone != portrait, and Camera == Back)
                 PHONE_IS_PORTRAIT = false;
                 parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
             } catch (Exception e) {
                 useWebcam = false; this.useWebcam = false; // Default to not using webcam.
                 opMode.telemetry.addData("Webcam"," not detected");
             }
        }

        if (!useWebcam) {
            if (useBackCamera) {
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
            } else {
                parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            }
        }
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if(useWebcam) {
            vuforia.enableConvertFrameToBitmap();
            AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        }

        if(useVuforiaTrackables) {
            initializeVuforiaTrackingGeometry(parameters);
            activateVuforiaTracking(); // Do we want to start Tracking on initialization?
        }

        // Initialize TensorFlow, if possible.
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod(useTensorFlowMonitor);
            activateTensorFlow(); // Do we want to start TF on initialization?
        } else {
            opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

    }


    public void activateVuforiaTracking() {
        targetsSkystone.activate();
    }

    public OpenGLMatrix getLastLocation() {
        return lastLocation;
    }

    public void activateTensorFlow() {
        if (tfod != null) {
            tfod.activate();
        }
    }

    /*
    *   Initializes vuforia trackables and sets the camera and field geometry.
    * */
    private void initializeVuforiaTrackingGeometry(VuforiaLocalizer.Parameters parameters) {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromFile("/sdcard/FIRST/Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkystone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (parameters.cameraDirection == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

    }




    /**
     * Display the pose of the vuMark, if detected.
     */
    public void updateVuMarkPose() {
        if(useVuforiaTrackables) {

            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    opMode.telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }


            // Provide feedback as to where the robot is located (if we know).
            //TODO Separate detections of Skystone from all other targets, since the skystone coordinates are relative.
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                // Note that reported pitch increases downward, since Y is to the robot's left.
                opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                opMode.telemetry.addData("Visible Target", "none");
            }
            //opMode.telemetry.update();
        }
    }

    public MecanumNavigation.Navigation2D getPositionNav2D() {
        if (lastLocation != null) {
            return new MecanumNavigation.Navigation2D(
                    lastLocation.getTranslation().get(0)/mmPerInch,
                    lastLocation.getTranslation().get(1)/mmPerInch,
                    Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS).thirdAngle);
        } else {
                return new MecanumNavigation.Navigation2D(0,0,0);
        }
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod(boolean useTensorFlowMonitor) {
        tfTimer = new ElapsedTime();
        TFObjectDetector.Parameters tfodParameters;
        if (useTensorFlowMonitor) {
            int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        } else {
            tfodParameters = new TFObjectDetector.Parameters();
        }

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public void updateTensorFlow(boolean portrait) {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                pastRecognitions = updatedRecognitions;
                tfTimer.reset();
                if (updatedRecognitions.size() == 3) {
                    int skystoneX1 = -1;
                    int skystoneX2 = -1;
                    int stone1 = -1;
                    int stone2 = -1;
                    int stone3 = -1;
                    int stone4 = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            skystoneX1 = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        } else if (stone1 == -1) {
                            stone1 = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        } else {
                            stone2 = portrait ? (int) recognition.getLeft() : (int) recognition.getTop();
                        }
                    }
                    if (skystoneX1 != -1 && stone1 != -1 && stone2 != -1) {
                        if (skystoneX1 < stone1 && skystoneX1 < stone2) {
                            setSkystonePosition(SkystoneSets.LEFT);
                        } else if (skystoneX1 > stone1 && skystoneX1 > stone2) {
                            setSkystonePosition(SkystoneSets.RIGHT);
                        } else {
                            setSkystonePosition(SkystoneSets.CENTER);
                        }
                    }
                }
            }
        }


    }

    public void displayTensorFlowDetectionSample() {
        opMode.telemetry.addData("# Object Detected", pastRecognitions.size());

        if (skystonePositions == SkystoneSets.LEFT) {
            opMode.telemetry.addData("Skystone Position", "Left");
        } else if (skystonePositions == SkystoneSets.RIGHT) {
            opMode.telemetry.addData("Skystone Position", "Right");
        } else {
            opMode.telemetry.addData("Skystone Position", "Center");
        }
    }

    public void setSkystonePosition(SkystoneSets skystonePosition) {
        if (skystonePosition != SkystoneSets.UNKNOWN) {
            this.skystonePositions = skystonePosition;
        }
    }

    public void displayTensorFlowDetections() {
        if (tfod != null) {
            if (pastRecognitions != null && tfTimer.seconds() <= 1) {
                opMode.telemetry.addData("Gold Location:",skystonePositions.toString());
                int number_of_recognitions = pastRecognitions.size();
                opMode.telemetry.addData("# Object Detected", number_of_recognitions);
                if (number_of_recognitions > 0) {
                    for (Recognition recognition : pastRecognitions) {
                        opMode.telemetry.addData(recognition.getLabel(),
                        "{Left, Right, Top, Bottom} = %.0f, %.0f, %.0f, %.0f",
                                recognition.getLeft(),recognition.getRight(), recognition.getTop(), recognition.getBottom());
                    }
                }

            }
        }
    }

    public Color.Mineral identifyMineral(SkystoneIdentificationLocation idLocation) {
        Color.Mineral detectedMineralColor = Color.Mineral.UNKNOWN;

        if (pastRecognitions != null && pastRecognitions.size() > 0) {
            int closestDetectionIndex = 0; // Assume 1st is closest.
            double closestDetectionDistance = 1e6; //
            double currentDistance;

            int imageHeight = pastRecognitions.get(0).getImageHeight();
            int imageWidth = pastRecognitions.get(0).getImageWidth();

            int targetX;
            int targetY;
            double detectionX;
            double detectionY;

            if (idLocation == SkystoneIdentificationLocation.CENTER) {
                // Find image center
                targetX = imageWidth / 2;
                targetY = imageHeight / 2;
            } else if (idLocation == SkystoneIdentificationLocation.BOTTOM) {
                // Find bottom center
                targetX = imageWidth /2;
                targetY = imageHeight; // Bottom is max pixel value.
            } else { // Default to image center.
                targetX = imageWidth / 2;
                targetY = imageHeight / 2;
            }

            if (pastRecognitions != null && pastRecognitions.size() > 0 && tfTimer.seconds() <= 1) {
                // Fresh detection exist:
                int detectionIndex = -1;
                for (Recognition recognition : pastRecognitions) {
                    // find distance to target location.
                    detectionIndex++;
                    detectionX = (recognition.getLeft() + recognition.getRight()) / 2;
                    detectionY = (recognition.getTop() + recognition.getBottom()) / 2;
                    currentDistance = Math.sqrt(Math.pow(targetX - detectionX, 2) + Math.pow(targetY - detectionY, 2));
                    if (currentDistance < closestDetectionDistance) {
                        closestDetectionDistance = currentDistance;
                        closestDetectionIndex = detectionIndex;
                    }
                }
                // After finding closest mineral, identify it and return its color.
                String idLabel = pastRecognitions.get(closestDetectionIndex).getLabel();
                if (idLabel == LABEL_FIRST_ELEMENT) {
                    detectedMineralColor = Color.Mineral.Skystone;
                } else if (idLabel == LABEL_SECOND_ELEMENT) {
                    detectedMineralColor = Color.Mineral.Stone;
                } else {
                    detectedMineralColor = Color.Mineral.UNKNOWN;
                }

                // Telemetry
                if (detectedMineralColor != Color.Mineral.UNKNOWN) {
                    opMode.telemetry.addData("Found", idLabel);
                    opMode.telemetry.addData("Mineral distance to target", closestDetectionDistance);
                } else {
                    opMode.telemetry.addData("No Mineral Detected in target location", "");
                }

            }

            return  detectedMineralColor;
        }

        return detectedMineralColor;
    }

    /**
     * Sample one frame from the Vuforia stream and write it to a .PNG image file on the robot
     * controller in the /sdcard/FIRST/data directory. The images can be downloaded using Android
     * Studio's Device File Explorer, ADB, or the Media Transfer Protocol (MTP) integration into
     * Windows Explorer, among other means. The images can be useful during robot design and calibration
     * in order to get a sense of what the camera is actually seeing and so assist in camera
     * aiming and alignment.
     */
    void captureFrameToFile() {
        vuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), new Consumer<Frame>()
        {
            @Override public void accept(Frame frame)
            {
                Bitmap bitmap = vuforia.convertFrameToBitmap(frame);
                if (bitmap != null) {
                    File file = new File(captureDirectory, String.format(Locale.getDefault(), "VuforiaFrame-%d.png", captureCounter++));
                    try {
                        FileOutputStream outputStream = new FileOutputStream(file);
                        try {
                            bitmap.compress(Bitmap.CompressFormat.PNG, 100, outputStream);
                        } finally {
                            outputStream.close();
                            opMode.telemetry.log().add("captured %s", file.getName());
                        }
                    } catch (IOException e) {
                        RobotLog.ee(TAG, e, "exception in captureFrameToFile()");
                    }
                }
            }
        }));
    }


}
