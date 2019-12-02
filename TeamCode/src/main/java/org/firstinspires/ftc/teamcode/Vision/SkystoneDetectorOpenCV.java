package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.opencvSkystoneDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.teamcode.Utilities.Color;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetectorOpenCV {

    RobotHardware opmode;
    HardwareMap hardwareMap;
    Color.Ftc teamColor;
    public OpenCvCamera webcam;


    private final int rows = 640;
    private final int cols = 480;



    public SkystoneDetectorOpenCV(RobotHardware opmode, Color.Ftc teamColor) {
       this.teamColor = teamColor;
       this.opmode = opmode;
       this.hardwareMap = opmode.hardwareMap;
       init(new AveragingPipeline());
    }

    public void init(OpenCvPipeline openCvPipeline) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(openCvPipeline);
        // If the resolution specified is not supported by the camera, an error will be thrown.
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);
    }

}


