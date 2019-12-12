package org.firstinspires.ftc.teamcode.Vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;

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
    public ArrayList<NormalizedRectangle> scanRegions = new ArrayList<>();
    NormalizedRectangle backgroundRegion = new NormalizedRectangle();
    public NormalizedValue lineThickness = new NormalizedValue(0.01);
    public NormalizedValue markerSize = new NormalizedValue(0.03);


    public AveragingPipeline() {
        double yPosition = 0.55;
        double[] normalizedSize = {0.10,0.08};
        double[] backgroundSize = {0.7,0.12};
        scanRegions.add(new NormalizedRectangle(0.25,yPosition,normalizedSize[0],normalizedSize[1]));
        scanRegions.add(new NormalizedRectangle(0.5,yPosition,normalizedSize[0],normalizedSize[1]));
        scanRegions.add(new NormalizedRectangle(0.75,yPosition,normalizedSize[0],normalizedSize[1]));
        backgroundRegion = new NormalizedRectangle(0.5,yPosition,backgroundSize[0],backgroundSize[1]);
        this.lineThickness = new NormalizedValue(0.005);
        this.markerSize = new NormalizedValue(0.03);
    }

    public AveragingPipeline(ArrayList<NormalizedRectangle> scanRegions, NormalizedRectangle backgroundRegion, NormalizedValue lineThickness, NormalizedValue markerSize) {
        // Warning: these are copied by reference
        this.scanRegions = scanRegions;
        this.backgroundRegion = backgroundRegion;
        this.lineThickness = lineThickness;
        this.markerSize = markerSize;
    }

    // Infer background size from normalized regions.
    public AveragingPipeline(ArrayList<NormalizedRectangle> scanRegions) {
        // Warning: these are copied by reference
        this.scanRegions = scanRegions;
        // Default values for line and marker sizes.
        this.lineThickness = new NormalizedValue(0.005);
        this.markerSize = new NormalizedValue(0.03);

        // Calculate a background rectangle which will overlap the sample regions.
        Double[] minXY = {1.0,1.0};
        Double[] maxXY = {0.0,0.0};
        ArrayList<Double> xPosition = new ArrayList<>();
        ArrayList<Double> yPosition = new ArrayList<>();
        for(NormalizedRectangle scanRegion: scanRegions) {
            minXY[0] = Math.min(minXY[0],scanRegion.centerXY.getNormalizedX()-scanRegion.sizeXY.getNormalizedX()/2.0);
            maxXY[0] = Math.max(maxXY[0],scanRegion.centerXY.getNormalizedX()+scanRegion.sizeXY.getNormalizedX()/2.0);
            minXY[1] = Math.min(minXY[1],scanRegion.centerXY.getNormalizedY()-scanRegion.sizeXY.getNormalizedY()/2.0);
            maxXY[1] = Math.max(maxXY[1],scanRegion.centerXY.getNormalizedY()+scanRegion.sizeXY.getNormalizedY()/2.0);
            xPosition.add(scanRegion.centerXY.getNormalizedX());
            yPosition.add(scanRegion.centerXY.getNormalizedY());
        }
        // Calculate average x,y center position:
        double[] averageCenterXY = {0.0,0.0};
        for(int i = 0; i < xPosition.size(); ++i) {
           averageCenterXY[0] += xPosition.get(i);
           averageCenterXY[1] += yPosition.get(i);
        }
        averageCenterXY[0] = averageCenterXY[0] / (double) xPosition.size();
        averageCenterXY[1] = averageCenterXY[1] / (double) yPosition.size();
        double[] sizeXY = {maxXY[0] - minXY[0],maxXY[1] - minXY[1]};
        this.backgroundRegion = new NormalizedRectangle(averageCenterXY[0], averageCenterXY[1], sizeXY[0], sizeXY[1]);
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
    private Mat subMat;

    private int min;
    private int minIndex;
    private ArrayList<Integer> avgArray = new ArrayList<>();
    private int backgroundAvg = 0;



    public void getStatus() {
        for(Integer thisAvg : avgArray) {
            System.out.print(thisAvg.toString() + ", ");
        }
        System.out.println();
        //return new ArrayList<>(avg1,avg2,avg3,avgBackground);
    }

    public ArrayList<Integer> getAvgArray() {
        return avgArray;
    }

    // Output
    private SkystoneRelativeLocation skystoneRelativeLocation = SkystoneRelativeLocation.UNKNOWN;
    private int position = 0; // output position
    public int getPosition() {
        return position;
    }

    @Override
    public SkystoneRelativeLocation getSkystoneRelativeLocation() {
        // TODO: Ensure the enum locations are correct, based on the positions.
        switch (position) {
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


    @Override
    public Mat processFrame(Mat input)
    {
        lastInputImage = input.clone();


        // Convert the image from RGB to YCrCb
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb channel from the image
        Core.extractChannel(YCrCb, Cb, 2);

        // Colors for drawing regions
        Scalar scanRectangleColor = new Scalar(0, 0, 255);
        Scalar backgroundRectangleColor = new Scalar(255, 0, 0);

        ArrayList<Integer> tempAvgArray = new ArrayList<>();
        for(NormalizedRectangle normalizedRectangle: scanRegions) {
            // Select Sample Area
            subMat = Cb.submat(normalizedRectangle.getOpenCVRectangle(input));
            // Average the sample areas and store in array
            tempAvgArray.add((int)Core.mean(subMat).val[0]);
            // Draw rectangles around the sample areas
            Imgproc.rectangle(input, normalizedRectangle.getOpenCVRectangle(input), scanRectangleColor, (int) Math.ceil(lineThickness.getPixelScaledValue(input)));
        }
        avgArray = tempAvgArray;

        // Draw rectangles around the background area
        Imgproc.rectangle(input, backgroundRegion.getOpenCVRectangle(input), backgroundRectangleColor, (int) Math.ceil(lineThickness.getPixelScaledValue(input)));

        // Figure out which sample zone had the lowest contrast from blue (lightest color)
        min = Collections.min(avgArray);

        minIndex = 0;
        for(Integer average: avgArray) {
            if(min == average) {
                break;
            } else {
                ++minIndex;
            }
        }
        // Draw a circle on the detected skystone
        Scalar markerColor = new Scalar(255,52,235);
        Imgproc.circle(input, scanRegions.get(minIndex).centerXY.getOpenCvPoint(input),
                (int) Math.ceil(markerSize.getPixelScaledValue(input)), markerColor, -1);


        // Free the allocated submat memory
        subMat.release();
        subMat = null;

        return input;
    }

}