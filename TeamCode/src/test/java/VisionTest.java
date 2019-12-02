import static com.google.common.truth.Truth.assertThat;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints.LabeledWaypoint;
import org.firstinspires.ftc.teamcode.Vision.SkystoneDetectorOpenCV;
import org.firstinspires.ftc.teamcode.WebcamExample;
import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotHardware.StartPosition.FIELD_LOADING;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.*;

public class VisionTest {

    // Load x64 OpenCV Library dll
    static {
        try {
            System.load("C:/opencv/build/java/x64/opencv_java343.dll");
            // https://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.4.3/
        } catch (UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
            System.err.println("For windows 10, download OpenCV Library from:");
            System.err.println("https://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.4.3/");
            System.err.println("And extract to your C:\\ drive");

            System.exit(1);
        }
    }

    String IMAGE_READ_PATH = "./TestData/openCV_input/";
    String IMAGE_WRITE_PATH = "./TestData/openCV_output/";


    Mat input = new Mat();

    @Before
    public void initialize() {
        String filePath = IMAGE_READ_PATH + "iphone7_27inches_by_2.5_up_inches.jpg";
        input = Imgcodecs.imread(filePath);
    }

    @Test
    public void imageRead() {
        System.out.println("Input Width: " + input.width() + "   Input Height: " + input.height());
        assertThat(input.width()).isAtLeast(1);
        assertThat(input.height()).isAtLeast(1);
    }

    @Test
    public void imageWrite() {
        String writePath = IMAGE_WRITE_PATH + "outputTestImage.jpg";
        Imgcodecs.imwrite(writePath, input);
        File outputFile = new File(writePath);
        assertThat(outputFile.exists()).isTrue();
    }

    @Test
    public void colorThresholding() {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
        Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

        //b&w
        Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 152, 255, Imgproc.THRESH_BINARY_INV);

        //outline/contour
        Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        yCbCrChan2Mat.copyTo(all);//copies mat object
        Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours

        Imgcodecs.imwrite(IMAGE_WRITE_PATH + "yCbCr.jpg", yCbCrChan2Mat);
        Imgcodecs.imwrite(IMAGE_WRITE_PATH + "threshold.jpg", thresholdMat);
        Imgcodecs.imwrite(IMAGE_WRITE_PATH + "all.jpg", all);
    }


    @Test
    public void testSkystoneDetectorPipeline() {
        SkystoneDetectorOpenCV skystoneDetectorOpenCV = new SkystoneDetectorOpenCV(Color.Ftc.BLUE);
        SkystoneDetectorOpenCV.AveragingPipeline testPipeline = skystoneDetectorOpenCV.getPipelineForTesting();
        Mat outputMat = testPipeline.processFrame(input);
//        Imgproc.circle(outputMat, new Point(outputMat.width()/2,outputMat.height()/2),
//                50, new Scalar(225, 52, 235), 4);
        Imgcodecs.imwrite(IMAGE_WRITE_PATH + "pipeline.jpg",outputMat);
    }
}
