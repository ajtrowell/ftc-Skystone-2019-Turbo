package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

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

public class AveragingPipeline extends TernarySkystonePipeline
{

    // Relative Sampling locations. Values normalized to image size, from [0,1].
    private SampleLocationsNormalized normalizedLocations;

    public SampleLocationsNormalized getNormalizedLocations() {
        return normalizedLocations;
    }

    public void setNormalizedLocations(SampleLocationsNormalized sampleLocationsNormalized) {
        this.normalizedLocations = sampleLocationsNormalized;
        pixelsScaled = false;
    }

    AveragingPipeline() {
        SampleLocationsNormalized visionNormalizedLocations =
                new SampleLocationsNormalized();
        visionNormalizedLocations.leftPosition = new Point(0.25,0.5);
        visionNormalizedLocations.centerPosition = new Point(0.5,0.5);
        visionNormalizedLocations.rightPosition = new Point(0.75,0.5);
        visionNormalizedLocations.blockSize = new Point(0.2, 0.2);
        visionNormalizedLocations.backgroundSize = new Point(0.6, 0.2);
        visionNormalizedLocations.lineThickness = 0.01;
        visionNormalizedLocations.markerSize = 0.1;
        this.normalizedLocations = visionNormalizedLocations;

    }

    AveragingPipeline(SampleLocationsNormalized sampleLocationsNormalized) {
        this.normalizedLocations = sampleLocationsNormalized;
    }

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
    private Mat subMatBackground;
    private int max;
    private int avg1;
    private int avg2;
    private int avg3;
    private int avgBackground;



    // Sampling pixel locations
    private Point imageSizeScaling = new Point(0,0); // Not scaled by default.
    private boolean pixelsScaled = false;
    private Point skystone = new Point();
    private Point sub1PointA = new Point(185, 23); // -25 Stone2
    private Point sub1PointB = new Point(195, 33);
    private Point sub2PointA = new Point(185, 99); // -50 Stone3
    private Point sub2PointB = new Point(195, 109);
    private Point sub3PointA = new Point(185, 164); //-106 Stone4
    private Point sub3PointB = new Point(195, 174);
    private Point subBackgroundPointA = new Point(185, 164); //-106 Stone4
    private Point subBackgroundPointB = new Point(195, 174);

    private int lineThickness = 1;
    private int markerSize = 10;

    // Output
    // Demo2
    private int position = 0; // output position
    private SkystoneRelativeLocation skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;

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
        if(currentSize.equals(imageSizeScaling) && pixelsScaled) return;
        // else, resize as needed.

        int imageWidth = input.width();
        int imageHeight = input.height();
        int maxLength = Math.max(imageHeight,imageWidth);

        Point leftPosition = new Point(imageWidth * normalizedLocations.leftPosition.x, imageHeight * normalizedLocations.leftPosition.y);
        Point centerPosition = new Point(imageWidth * normalizedLocations.centerPosition.x, imageHeight * normalizedLocations.centerPosition.y);
        Point rightPosition = new Point(imageWidth * normalizedLocations.rightPosition.x, imageHeight * normalizedLocations.rightPosition.y);
        Point blockSize = new Point(imageWidth * normalizedLocations.blockSize.x, imageHeight * normalizedLocations.blockSize.y);
        Point backgroundSize = new Point(imageWidth * normalizedLocations.backgroundSize.x, imageHeight * normalizedLocations.backgroundSize.y);

        this.lineThickness = (int) Math.ceil(maxLength * normalizedLocations.lineThickness);
        this.markerSize = (int) Math.ceil(maxLength * normalizedLocations.markerSize);



        pixelsScaled = true;
        imageSizeScaling = new Point(input.width(), input.height());
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

    @Override
    public SkystoneRelativeLocation getSkystoneRelativeLocation() {
        // TODO: Ensure the enum locations are correct, based on the positions.
        switch (position) {
            case 0:
                skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
                break;
            case 1:
                skystoneRelativeLocation = SkystoneRelativeLocation.LEFT;
                break;
            case 2:
            default:
                skystoneRelativeLocation = SkystoneRelativeLocation.CENTER;
                break;
            case 3:
                skystoneRelativeLocation = SkystoneRelativeLocation.RIGHT;
                break;
        }

        return skystoneRelativeLocation;
    }
}