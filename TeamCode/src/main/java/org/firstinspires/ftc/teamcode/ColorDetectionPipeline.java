package org.firstinspires.ftc.teamcode;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorDetectionPipeline extends OpenCvPipeline {

    public double highRowHeight = 0.0;
    public double lowRowHeight = 0.1;
    private Mat targetRowViewerMat;
    private Mat distanceViewerMat;
    private Mat input;
    private Mat inputSubmat;
    private Mat squishedInput;
    private double parsedRowHeight;
    public int viewportStage = 0;
    private int viewerHeight = 10;
    private Size targetRes;
    public double[] targetColor;
    public double[] readColor;
    private double avgPos = 0;
    public boolean debug = true;
    public int highRow;
    public int lowRow;

    public ColorDetectionPipeline(Size targetRes, double highTargetRowHeight, double lowTargetRowHeight, double[] targetColor) {
        highRowHeight = highTargetRowHeight;
        lowRowHeight = lowTargetRowHeight;
        this.targetColor = targetColor;
        this.targetRes = targetRes;
        this.input = new Mat(targetRes, CvType.CV_8UC4);
        Size rowSize = new Size((int) targetRes.width, viewerHeight);
        this.distanceViewerMat = new Mat(rowSize, CvType.CV_8UC4);
        this.targetRowViewerMat = new Mat(rowSize, CvType.CV_8UC4);
        this.squishedInput = new Mat(1, (int) targetRes.width, CvType.CV_8UC4);
    }

    @Override
    public void onViewportTapped()
    {
        viewportStage += 1;
    }

    public void setRowHeight(double high, double low) {
        highRowHeight = high;
        lowRowHeight = low;
    }

    public void setTargetColor(double[] color) {
        this.targetColor = color;
    }

    @Override
    public Mat processFrame(Mat highresInput)
    {

        // downsample the input
        Imgproc.resize(highresInput, input, input.size(), 0, 0, Imgproc.INTER_AREA);
        highRow = (int) Math.round(highRowHeight * input.rows());
        lowRow = (int) Math.round(lowRowHeight * input.rows());
        // get the submat of the input
        inputSubmat = input.submat(highRow, lowRow, 0, input.cols());
        Imgproc.resize(inputSubmat, squishedInput, squishedInput.size(), 0, 0, Imgproc.INTER_AREA);


        double rowLength = squishedInput.cols();
        ArrayList<Double> weights = new ArrayList<Double>();
        ArrayList<Double> colorDistances = new ArrayList<Double>();
        double totalWeight = 0;
        double maxColorDistance = -100000;
        double minColorDistance = 100000;


        // find raw color distance for each pixel
        for (int column = 0; column < rowLength; column++) {
            double[] color = squishedInput.get(0, column);
            if (debug) {
                for (int row = 0; row < viewerHeight; row++) {
                    targetRowViewerMat.put(row, column, color.clone());
                }
            }

            double colorDistance = Math.sqrt(
                    Math.pow(color[0] - targetColor[0], 2)
                            + Math.pow(color[1] - targetColor[1], 2)
                            + Math.pow(color[2] - targetColor[2], 2)
            );
            colorDistances.add(colorDistance);
            maxColorDistance = Math.max(colorDistance, maxColorDistance);
            minColorDistance = Math.min(colorDistance, minColorDistance);
        }

        // find weights for each pixel
        final double colorDistanceRange = maxColorDistance - minColorDistance;
        double middleColumn = rowLength/2.0;
        boolean onChunk = false;
        boolean onSmallChunk = false;
        double smallestChunkDistance = 10000;
        double currentSmallestChunkStart = 0;
        double currentSmallestChunkEnd = 0;
        for (int i = 0; i < rowLength; i++) {
            double pos = (double) i;
            double colorDistance = colorDistances.get(i);
            double normalizedColorDistance = (colorDistance - minColorDistance)/colorDistanceRange;
            if (debug) {
                double viewerColor = normalizedColorDistance < 0.25 ? 255 : 0;
                for (int row = 0; row < viewerHeight; row++) {
                    distanceViewerMat.put(row, i, new double[]{viewerColor, viewerColor, viewerColor, 255});
                }
            }
            if (normalizedColorDistance < 0.25) {
                double chunkDistance = pos - middleColumn;
                if (Math.abs(chunkDistance) < Math.abs(smallestChunkDistance) && !onChunk) {
                    smallestChunkDistance = chunkDistance;
                    currentSmallestChunkStart = pos;
                    onSmallChunk = true;
                }
                onChunk = true;
            } else if (onChunk) {
                onChunk = false;
                if (onSmallChunk) {
                    onSmallChunk = false;
                    currentSmallestChunkEnd = pos - 1.0;
                    double chunkDistance = currentSmallestChunkEnd - middleColumn;
                    if (Math.abs(chunkDistance) < Math.abs(smallestChunkDistance)) {
                        smallestChunkDistance = chunkDistance;
                    }
                }
            }
        }
        if (onSmallChunk) {
            onSmallChunk = false;
            currentSmallestChunkEnd = rowLength - 1.0;
            double chunkDistance = currentSmallestChunkEnd - middleColumn;
            if (Math.abs(chunkDistance) < Math.abs(smallestChunkDistance)) {
                smallestChunkDistance = chunkDistance;
            }
        }

        avgPos = ((currentSmallestChunkStart + currentSmallestChunkEnd)/2.0 - middleColumn)/middleColumn;

        if (debug) {
            int avgPosPixel = (int) ((avgPos + 1.0)/2.0 * rowLength);
            for (int row = 0; row < viewerHeight; row++) {
                distanceViewerMat.put(row, avgPosPixel, new double[]{255, 0, 0, 255});
                targetRowViewerMat.put(row, avgPosPixel, new double[]{255, 0, 0, 255});
            }
        }

        switch (viewportStage) {
            case 0: return highresInput;
            case 1: return input;
            case 2: return inputSubmat;
            case 3: return targetRowViewerMat;
            case 4: return distanceViewerMat;
            case 5: viewportStage = 0;
                return highresInput;
        }
        return distanceViewerMat;
    }

    public double getColorDir() {
        return avgPos;
    }

    public Mat getInput() {
        return input;
    }

    public Mat getDistanceViewerMat() {
        return distanceViewerMat;
    }

}
