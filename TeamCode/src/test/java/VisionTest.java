import static com.google.common.truth.Truth.assertThat;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints;
import org.firstinspires.ftc.teamcode.Utilities.Waypoints.LabeledWaypoint;
import org.junit.Before;
import org.junit.Test;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import static org.firstinspires.ftc.teamcode.RobotHardware.StartPosition.FIELD_LOADING;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.LocationLoading.*;
import static org.firstinspires.ftc.teamcode.Utilities.Waypoints.*;

public class VisionTest {

    // Load x64 OpenCV Library dll
    static {
        try {
            System.load("C:/opencv/build/java/x64/opencv_java343.dll");
        } catch(UnsatisfiedLinkError e) {
            System.err.println("Native code library failed to load.\n" + e);
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
        Imgcodecs.imwrite(writePath,input);
        File outputFile = new File(writePath);
        assertThat(outputFile.exists()).isTrue();
    }

}
