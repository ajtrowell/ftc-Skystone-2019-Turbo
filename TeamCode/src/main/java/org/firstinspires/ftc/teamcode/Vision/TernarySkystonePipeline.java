package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;

abstract class TernarySkystonePipeline extends OpenCvPipeline {

    public abstract SkystoneRelativeLocation getSkystoneRelativeLocation();

    /**
     * Data Class for storing the normalized sizes and locations
     * of regions to scan for the skystone.
     * Measurements are normalized to the height and width of the input image.
      */
    public static class SampleLocationsNormalized {

        // Center locations of 3 sample regions
        public Point leftPosition;
        public Point centerPosition;
        public Point rightPosition;
        // Sample region size
        public Point blockSize;
        public Point backgroundSize;
        public double lineThickness = -1;
        public double markerSize = -1;


        SampleLocationsNormalized() {

        }


        SampleLocationsNormalized(Point leftPosition, Point centerPosition, Point rightPosition,
                                  Point blockSize, Point backgroundSize,
                                  double lineThickness, double markerSize) {
            this.leftPosition = leftPosition;
            this.centerPosition = centerPosition;
            this.rightPosition = rightPosition;
            this.blockSize = blockSize;
            this.backgroundSize = backgroundSize;
            this.lineThickness = lineThickness;
            this.markerSize = markerSize;
        }

        /**
         * Validate all data values.
         * None should be null, all should be [0,1],
         * and half the sizes plus any position should not be outside [0,1]
          * @return
         */
        public boolean isValid() {
            boolean notNull;
            boolean withinRange;
            boolean compoundRange;

            // Not null
            notNull =
                    (leftPosition != null) &&
                    (centerPosition != null) &&
                    (rightPosition != null) &&
                    (blockSize != null) &&
                    (backgroundSize != null) &&
                    (lineThickness > 0) &&
                    (markerSize > 0);

            // Verify range of values, including compound ranges.
            withinRange = (leftPosition.x >= 0) && (leftPosition.x <= 1) && (leftPosition.y >= 0) && (leftPosition.y <=1) &&
            (rightPosition.x >= 0) && (rightPosition.x <= 1) && (rightPosition.y >= 0) && (rightPosition.y <=1) &&
            (blockSize.x >= 0) && (blockSize.x <= 1) && (blockSize.y >= 0) && (blockSize.y <=1) &&
            (backgroundSize.x >= 0) && (backgroundSize.x <= 1) && (backgroundSize.y >= 0) && (backgroundSize.y <=1) &&
            lineThickness >= 0 && lineThickness <=1 &&
            markerSize >= 0 && markerSize <=1;


            // Compound sizes
            double minX = Math.min(Math.min(leftPosition.x,centerPosition.x),rightPosition.x);
            double minY = Math.min(Math.min(leftPosition.y,centerPosition.y),rightPosition.y);
            double maxX = Math.max(Math.max(leftPosition.x,centerPosition.x),rightPosition.x);
            double maxY = Math.max(Math.max(leftPosition.y,centerPosition.y),rightPosition.y);
            double maxOffsetX = Math.max(blockSize.x,backgroundSize.x);
            double maxOffsetY = Math.max(blockSize.y,backgroundSize.y);

            compoundRange =
                ((minX - maxOffsetX) >= 0) &&
                ((maxX + maxOffsetX) <= 1) &&
                ((minY - maxOffsetY) >= 0) &&
                ((maxY + maxOffsetY) <= 1);

            return notNull && withinRange && compoundRange;
        }
    }


}
