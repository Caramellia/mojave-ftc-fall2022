/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.function.Supplier;

@TeleOp(name = "Pole Detection Test", group="Linear Opmode")
public class PoleDetectionTest extends BaseController {

    // rotation stuff
    private final double RIGHT_ANGLE = Math.PI/2.0;
    private int phase = 0;
    private final double MAX_SPEED_MULT = 0.45;
    private final double MAX_ACCEL_TIME = 0.5;
    private final double IN_TO_MM = 25.4;
    private final double FIELD_SIZE = 141.345 * IN_TO_MM;
    private final double TILE_SIZE = FIELD_SIZE/6.0;
    private final double SLOW_BEGIN_THRESHOLD = 3 * IN_TO_MM;
    private final double PHASE_CHANGE_THRESHOLD = 0.5 * IN_TO_MM;
    private final double ROTATION_PHASE_CHANGE_THRESHOLD = Math.toRadians(2.5);
    private double phaseStartTime = 0;
    private boolean phaseEndReached = false;
    private double phaseEndReachedTime = 0;
    private final boolean phaseChanged = false;
    private final boolean movementPhase = true;
    private VectorF desiredDisplacement = displacement;
    private final boolean goToNextPhase = true;
    private final double clawOpenTime = 0;

    float leftDst = (float) (-TILE_SIZE * 1.0);
    float fwdDst = (float) (-TILE_SIZE * 2.0);

    Supplier<Boolean> MovementPhaseCheck = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        return diff.magnitude() < PHASE_CHANGE_THRESHOLD;
    };

    Runnable MovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = (Math.min(runtime.seconds() - phaseStartTime, MAX_ACCEL_TIME) / MAX_ACCEL_TIME) // initial acceleration
                    * Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD // ending deceleration;
                    * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setLocalMovementVector(dir);
        } else {
            setLocalMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    // opencv
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    ColorDetectionPipeline colorDetectionPipeline;
    final float DECIMATION_LOW = 2;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int zone = -1;

    // dist: 62.5 inches

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() {

        initialize();
        colorDetectionPipeline = new ColorDetectionPipeline(new Size(50, 240), 0.6, 0.7, new double[]{255, 255, 0});

        camera.setPipeline(colorDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();
        runtime.reset();

        // MAIN LOOP
        while (opModeIsActive()) {
            double poleDir = colorDetectionPipeline.getColorDir();
            telemetry.addData("Pole Dir", poleDir);
            telemetry.addData("Input Mat", colorDetectionPipeline.getInput());
            telemetry.addData("Input Color", colorDetectionPipeline.getInput().get(0, 0));
            telemetry.addData("Input Color 0", colorDetectionPipeline.getInput().get(0, 0)[0]);
            telemetry.addData("Input Color 1", colorDetectionPipeline.getInput().get(0, 0)[1]);
            telemetry.addData("Input Color 2", colorDetectionPipeline.getInput().get(0, 0)[2]);
            telemetry.addData("Input Color 3", colorDetectionPipeline.getInput().get(0, 0)[3]);
            colorDetectionPipeline.viewportStage = (int) Math.min(Math.floor(runtime.seconds()/2.0), 5.0);
            if (runtime.seconds() > 11) {
                setLocalMovementVector(new VectorF((float) poleDir * 0.35f, 0, 0, 0));
            }
            telemetry.update();
            applyMovement();
            applyTargetRotation();
        }
    }

}

