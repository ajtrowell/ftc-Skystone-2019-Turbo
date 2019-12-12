package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Utilities.Color;

/**
 * Initializes a pipeline and starts streaming.
 * Pipeline is focused on image frame, providing relative position information (left, center, right)
 * SkystoneDetector can use teamColor and starting location information to provide more absolute
 * location information for the skystone, such as location index, where 0 is near the field center.
 */
public class SkystoneDetector {

    RobotHardware opmode;
    HardwareMap hardwareMap;
    Color.Ftc teamColor;
    public OpenCvCamera webcam;
    public AveragingPipeline averagingPipeline;

    private final int rows = 640;
    private final int cols = 480;


    public SkystoneDetector(RobotHardware opmode, Color.Ftc teamColor) {
       this.teamColor = teamColor;
       this.opmode = opmode;
       this.hardwareMap = opmode.hardwareMap;
       this.averagingPipeline = new AveragingPipeline();
       init(this.averagingPipeline);
    }

    public void init(AveragingPipeline averagingPipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(averagingPipeline);
        // If the resolution specified is not supported by the camera, an error will be thrown.
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
    }


    SkystoneRelativeLocation skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
    public SkystoneRelativeLocation getSkystoneRelativeLocation() {
        // TODO: Ensure the enum locations are correct, based on the positions.
        switch (averagingPipeline.getMinIndex()) {
            case 0:
                skystoneRelativeLocation = SkystoneRelativeLocation.LEFT;
                break;
            case 1:
                skystoneRelativeLocation = SkystoneRelativeLocation.CENTER;
                break;
            case 2:
                skystoneRelativeLocation = SkystoneRelativeLocation.RIGHT;
                break;
            default:
                skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
                break;
        }
        return skystoneRelativeLocation;
    }

    /*
    *****  Geometric Calculations *****
     */

    /**
     * Returns the normalized xy size of the stone scan region, from [0,1] as a fraction of the image size,
     * based on the camera FOV, the distance to the stone, and the size of the stone.
     * @param fov_degrees_xy Field of view of the camera in degrees.
     * @param distance Distance from camera. Units must be the same as sizeXY.
     * @param sizeXY Size of block. Units must be the same as distance.
     * @return normalizedSizeXY array, both values scaled from [0,1].
     */
    public double[] getNormalizedSize(double[] fov_degrees_xy, double distance, double [] sizeXY) {
        double[] normalizedSizeXY = {0,0};
        for(int i=0; i<=1; ++i) {
            normalizedSizeXY[i] = 2.0 / fov_degrees_xy[i] * 180/Math.PI * Math.atan(sizeXY[i] / (2.0 * distance));
        }
        return normalizedSizeXY;
    }

    /**
     * Returns normalizedPositionXY array, with values [0,1], indicating the position of the center
     * of the blocks on the camera image, where 0 refers to the minimum pixel location, and 1 refers
     * to the maximum pixel location.  {.5,.5} would reference the center of the image.
     * @param fov_degrees_xy Field of view of the camera in degrees.
     * @param relativePositionXY Block position relative to camera, X out of camera, Y to left. Units irrelevant.
     * @return normalizedPositionXY array, [0,1] position on image, in camera viewport. {0.5,0.5} is center of image.
     */
    public double[] getNormalizedPosition(double[] fov_degrees_xy, double[] relativePositionXY) {
        double[] normalizedPositionXY = {0,0};
        for(int i=0; i<=1; ++i) {
            normalizedPositionXY[i] = 0.5 - Math.atan2(relativePositionXY[1],relativePositionXY[0]) * (180 / Math.PI) /fov_degrees_xy[i];
        }
        return normalizedPositionXY;
    }




}


