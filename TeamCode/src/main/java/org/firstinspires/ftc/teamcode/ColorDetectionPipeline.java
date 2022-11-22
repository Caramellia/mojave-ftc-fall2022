package org.firstinspires.ftc.teamcode;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ColorDetectionPipeline extends OpenCvPipeline {

    private Mat targetRowViewerMat;
    private Mat distanceViewerMat;
    private Mat input;
    private double parsedRowHeight;
    private int viewportStage = 0;
    private int maxViewportStage = 1;
    private Size targetRes;
    private double[] targetColor;
    public double[] readColor;
    private double avgPos = 0;

    public ColorDetectionPipeline(Size targetRes, double targetRowHeight, double[] targetColor) {
        this.parsedRowHeight = targetRowHeight;
        this.targetColor = targetColor;
        this.targetRes = targetRes;
        this.input = new Mat(targetRes, CvType.CV_8UC4);
        Size rowSize = new Size((int) targetRes.width, 1);
        this.distanceViewerMat = new Mat(rowSize, CvType.CV_8UC4);
        this.targetRowViewerMat = new Mat(rowSize, CvType.CV_8UC4);
    }

    @Override
    public void onViewportTapped()
    {
        viewportStage++;
        if (viewportStage > maxViewportStage) {
            viewportStage = 0;
        }
    }

    @Override
    public Mat processFrame(Mat highresInput)
    {

        // downsample the input
        Imgproc.resize(highresInput, input, input.size(), 0, 0, Imgproc.INTER_AREA);

        int parsedRow = (int) Math.round(parsedRowHeight * input.rows());

        double rowLength = input.cols();
        double middleColumn = rowLength/2.0;
        ArrayList<Double> weights = new ArrayList<Double>();
        ArrayList<Double> colorDistances = new ArrayList<Double>();
        double totalWeight = 0;
        double maxColorDistance = -Double.MAX_VALUE;
        double minColorDistance = Double.MAX_VALUE;


        // find raw color distance for each pixel
        for (int column = 0; column < rowLength; column++) {
            double[] color = input.get(parsedRow, column);
            for (int row = 0; row < 100; row++) {
                targetRowViewerMat.put(row, column, color);
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
        for (int i = 0; i < rowLength; i++) {
            double normalizedColorDistance = (colorDistances.get(i) - minColorDistance)/colorDistanceRange;
            double weight = normalizedColorDistance < 0.5 ? (1.0 - normalizedColorDistance) : 0.0;
            totalWeight += weight;
            weights.add(weight);
            for (int row = 0; row < 100; row++) {
                //distanceViewerMat.put(row, i, new int[]{(int) (weight * 255), (int) (weight * 255), (int) (weight * 255), (int) (255 * 0.5)});
            }
        }



        // find the weighted average pos
        double tempAvgPos = 0.0;
        for (int i = 0; i < rowLength; i++) {
            double pos = (i - middleColumn)/middleColumn;
            double weight = weights.get(i)/totalWeight;
            tempAvgPos += pos * weight;
        }
        // silly goose! the weights are already normalized so that the total is 1
        //tempAvgPos /= rowLength;
        avgPos = tempAvgPos;
        int avgPosPixel = (int) Math.floor((avgPos + 1.0) * weights.size());
        //distanceViewerMat.put(0, avgPosPixel, new int[]{255, 0, 0, 0});

        /*switch (viewportStage) {
            case 0: return input;
            case 1: return targetRowViewerMat;
            case 2: return distanceViewerMat;
        }*/
        return targetRowViewerMat;
        //return distanceViewerMat;
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
