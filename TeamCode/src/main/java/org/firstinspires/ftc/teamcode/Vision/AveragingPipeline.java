package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * An example image processing pipeline to be run upon receipt of each frame from the camera.
 * Note that the processFrame() method is called serially from the frame worker thread -
 * that is, a new camera frame will not come in while you're still processing a previous one.
 * In other words, the processFrame() method will never be called multiple times simultaneously.
 *
 * However, the rendering of your processed image to the viewport is done in parallel to the
 * frame worker thread. That is, the amount of time it takes to render the image to the
 * viewport does NOT impact the amount of frames per second that your pipeline can process.
 *
 * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
 * frame worker thread. This should not be a problem in the vast majority of cases. However,
 * if you're doing something weird where you do need it synchronized with your OpMode thread,
 * then you will need to account for that accordingly.
 */

public class AveragingPipeline extends OpenCvPipeline
{
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */

    // CV intermediate products
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    private Mat subMat1;
    private Mat subMat2;
    private Mat subMat3;
    private int max;
    private int avg1;
    private int avg2;
    private int avg3;

    // Relative Sampling locations. Values normalized to image size, from [0,1].
    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};


    // Sampling pixel locations
    private Point imageSizeScaling = new Point(0,0); // Not scaled by default.
    private Point skystone = new Point();
    private Point sub1PointA = new Point(185, 23); // -25 Stone2
    private Point sub1PointB = new Point(195, 33);
    private Point sub2PointA = new Point(185, 99); // -50 Stone3
    private Point sub2PointB = new Point(195, 109);
    private Point sub3PointA = new Point(185, 164); //-106 Stone4
    private Point sub3PointB = new Point(195, 174);

    // Output
    // Demo2
    private int position = 0; // output position

    public int getPosition() {
        return position;
    }

    /**
     * If the pixel size isn't scaled to the current image size, use normalized sizes and
     * image size to calculate sample regions in pixels.
     * @param input
     */
    private void scaleSamplingLocationToImageSize(Mat input) {
        Point currentSize = new Point(input.width(),input.height());
        if(currentSize.equals(imageSizeScaling)) return;


    }


    @Override
    public Mat processFrame(Mat input)
    {
        // Ensure that pixel locations are scaled to the current input image resolution.
        scaleSamplingLocationToImageSize(input);

        // Convert the image from RGB to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel from the image
        Core.extractChannel(YCrCb, Cb, 2);

        // The the sample areas fromt the Cb channel
        subMat1 = Cb.submat(new Rect(sub1PointA, sub1PointB));
        subMat2 = Cb.submat(new Rect(sub2PointA, sub2PointB));
        subMat3 = Cb.submat(new Rect(sub3PointA, sub3PointB));

        // Average the sample areas
        avg1 = (int)Core.mean(subMat1).val[0]; // Stone2
        avg2 = (int)Core.mean(subMat2).val[0]; // Stone3
        avg3 = (int)Core.mean(subMat3).val[0]; // Stone4

        // Draw rectangles around the sample areas
        Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);

        // Figure out which sample zone had the lowest contrast from blue (lightest color)
        max = Math.max(avg1, Math.max(avg2, avg3));

        // Draw a circle on the detected skystone
        if(max == avg1) {
            skystone.x = (sub1PointA.x + sub1PointB.x) / 2;
            skystone.y = (sub1PointA.y + sub1PointB.y) / 2;
            Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
            position = 2;
            // Stone2
        } else if(max == avg2) {
            skystone.x = (sub2PointA.x + sub2PointB.x) / 2;
            skystone.y = (sub2PointA.y + sub2PointB.y) / 2;
            Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
            // Stone3
            position = 3;
        } else if(max == avg3) {
            skystone.x = (sub3PointA.x + sub3PointB.x) / 2;
            skystone.y = (sub3PointA.y + sub3PointB.y) / 2;
            Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
            // Stone1
            position = 1;
        } else {
            position = 1;
        }

        // Free the allocated submat memory
        subMat1.release();
        subMat1 = null;
        subMat2.release();
        subMat2 = null;
        subMat3.release();
        subMat3 = null;

        return input;
    }
}