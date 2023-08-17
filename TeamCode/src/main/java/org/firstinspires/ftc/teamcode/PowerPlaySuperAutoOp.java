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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.opencv.core.Size;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.function.Supplier;

@Autonomous(name = "Fast AutoOp Blue, Right", group = "Linear Opmode")
public class PowerPlaySuperAutoOp extends BaseAutoOp {

    // rotation stuff
    private int phase = 0;
    private double INITIAL_MAX_SPEED_MULT = 0.6;
    private double MAX_SPEED_MULT = INITIAL_MAX_SPEED_MULT;
    private double INITIAL_MAX_ACCEL_TIME = 0.5;
    private double MAX_ACCEL_TIME = INITIAL_MAX_ACCEL_TIME;
    private final double IN_TO_MM = 25.4;
    private final double FIELD_SIZE = 141.345 * IN_TO_MM;
    private final double TILE_SIZE = FIELD_SIZE/6.0;
    private final double SLOW_BEGIN_THRESHOLD = 3 * IN_TO_MM;
    private final double PHASE_CHANGE_THRESHOLD = 0.35 * IN_TO_MM;
    private final double ROTATION_PHASE_CHANGE_THRESHOLD = Math.toRadians(1.0);
    private VectorF desiredDisplacement = displacement;

    Supplier<Boolean> MovementPhaseCheck = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        return diff.magnitude() < PHASE_CHANGE_THRESHOLD;
    };

    Supplier<Boolean> RotationPhaseCheck = () -> {
        return Math.abs(targetRotation - rotation) < ROTATION_PHASE_CHANGE_THRESHOLD;
    };

    Runnable MovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = (Math.min(runtime.seconds() - phaseStartTime, MAX_ACCEL_TIME) / MAX_ACCEL_TIME) // initial acceleration
                    * Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD // ending deceleration;
                    * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) Math.max(speedMult, 0.075));
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    Runnable InitialMovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = (Math.min(runtime.seconds() - phaseStartTime, MAX_ACCEL_TIME) / MAX_ACCEL_TIME) // initial acceleration
                    //* Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD // ending deceleration;
                    * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    Runnable IntermediateMovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
        }
    };

    Runnable EndMovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacement);
        if (diff.magnitude() > 0.0) {
            double speedMult = Math.min(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD * MAX_SPEED_MULT;
            VectorF dir = diff.multiplied((float) (1.0 / diff.magnitude())).multiplied((float) speedMult);
            telemetry.addData("Movement Dir", dir);
            setWorldMovementVector(dir);
        } else {
            setWorldMovementVector(new VectorF(0, 0, 0, 0));
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
    public double[] coneColor = new double[]{0, 0, 255};
    //int zone = -1;

    // variable params
    float leftDst = (float) (-TILE_SIZE * 1.0);
    float fwdDst = (float) (-TILE_SIZE * 2.0);
    boolean placeFirstConeImmediately = false;
    int reps = 2;

    // dist: 62.5 inches

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void autoOpInitialize() {

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
        colorDetectionPipeline = new ColorDetectionPipeline(new Size(320, 240), 0.0, 0.1, new double[]{1, 1, 0});


        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // detect zone
        addPhase(() -> {
            clawOpen = false;
            setArmStage(1);
            telemetry.addData("Oh yeah baby!", "true");
            desiredDisplacement = new VectorF(leftDst, 0, 0, 0);
        }, () -> {
            setArmStage(1);
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if (zone == -1 && detections != null && detections.size() > 0) {
                zone = detections.get(0).id;
                camera.setPipeline(colorDetectionPipeline);
            }
        }, () -> zone != -1);

        float initialOffset = (float) (-2.5 * IN_TO_MM);
        float midLeftDst = (float) ((TILE_SIZE/2.0) * Math.signum(leftDst) * 1.0f);

        if (placeFirstConeImmediately) {
            addPhase(() -> {
                desiredDisplacement = new VectorF(midLeftDst, initialOffset, 0, 0);
            }, InitialMovementPhaseStep, MovementPhaseCheck);
            addPhase(() -> {
                desiredDisplacement = new VectorF(midLeftDst, initialOffset - 4.9f * (float) (IN_TO_MM), 0, 0);
            }, MovementPhaseStep, MovementPhaseCheck);

            // open claw
            addPhase(() -> {
                goalArmEncoderValue = -800;
            }, () -> {
            }, () -> (Math.abs(goalArmEncoderValue - realArmEncoderValue) < 20));
            addPhase(() -> {
                setClawOpen(true);
            }, () -> {
            }, () -> (runtime.seconds() - phaseStartTime > 0.65));
            addPhase(() -> {
                setArmStage(1);
            }, () -> {
            }, () -> (Math.abs(goalArmEncoderValue - realArmEncoderValue) < 50));

            // back up
            addPhase(() -> {
                desiredDisplacement = new VectorF(midLeftDst, initialOffset, 0, 0);
            }, InitialMovementPhaseStep, MovementPhaseCheck);
        }
        // lower arm, move to the left
        addPhase(() -> {
            desiredDisplacement = new VectorF(leftDst, initialOffset, 0, 0);
            setArmStage(0);
            goalArmEncoderValue = -210;
        }, placeFirstConeImmediately ? IntermediateMovementPhaseStep : InitialMovementPhaseStep, MovementPhaseCheck);

        // go forward
        addPhase(() -> desiredDisplacement = new VectorF(leftDst, fwdDst + initialOffset, 0, 0), IntermediateMovementPhaseStep, MovementPhaseCheck);


        reps += placeFirstConeImmediately ? 1 : 0;
        for (int i = 0; i < reps; i++) {

            if (!placeFirstConeImmediately || i > 0) {

                // go in front of pole and raise arm
                addPhase(() -> {
                    colorDetectionPipeline.setTargetColor(new double[]{255, 255, 0});
                    colorDetectionPipeline.highRowHeight = 0.0;
                    colorDetectionPipeline.lowRowHeight = 0.1;
                    setTargetRotation(0.0);
                    desiredDisplacement = new VectorF(midLeftDst, fwdDst + initialOffset, 0, 0);
                    setArmStage(3);
                }, EndMovementPhaseStep, () -> MovementPhaseCheck.get() && RotationPhaseCheck.get() && realArmEncoderValue < -600);

                // calibrate pos

                addPhase(() -> {
                    MAX_SPEED_MULT = 0.25;
                    MAX_ACCEL_TIME = 1.0;
                }, () -> {
                    double dir = colorDetectionPipeline.getColorDir();
                    double movement = Math.max(Math.abs(dir * 0.5), 0.05) * Math.signum(dir);
                    telemetry.addData("Mid Left Dst", midLeftDst);
                    telemetry.addData("Pole Dir", dir);
                    setMovementVectorRelativeToTargetOrientation(
                            new VectorF((float) movement, 0, 0, 0)
                    );
                    // "crash" protection
                    if (displacement.subtracted(desiredDisplacement).magnitude() > 10 * IN_TO_MM) {
                        goToPhase("park");
                    }
                }, () -> (Math.abs(colorDetectionPipeline.getColorDir()) < 0.025 || runtime.seconds() - phaseStartTime > 5)
                        && Math.abs(goalArmEncoderValue - realArmEncoderValue) < 40);

                // go further forward now that the arm is raised
                addPhase(() -> {
                    double displacementY = displacement.get(1);
                    setCurrentDisplacementAs(new VectorF(midLeftDst, (float) displacementY, 0, 0));
                    desiredDisplacement = new VectorF(midLeftDst, fwdDst + initialOffset - 4.3f * (float) IN_TO_MM, 0, 0);
                }, MovementPhaseStep, MovementPhaseCheck);

                // open claw
                addPhase(() -> {
                    goalArmEncoderValue = -2750;
                }, () -> {
                }, () -> (Math.abs(goalArmEncoderValue - realArmEncoderValue) < 20));
                addPhase(() -> {
                    setClawOpen(true);
                }, () -> {
                }, () -> (runtime.seconds() - phaseStartTime > 0.65));
                addPhase(() -> {
                    setArmStage(3);
                }, () -> {
                }, () -> (Math.abs(goalArmEncoderValue - realArmEncoderValue) < 50));

                // back up
                addPhase(() -> {
                    desiredDisplacement = new VectorF(midLeftDst, (float) (fwdDst + initialOffset - 1.0 * IN_TO_MM), 0, 0);
                }, InitialMovementPhaseStep, MovementPhaseCheck);

            }

            if (i < reps - 1) {
                // move to 0 pos
                addPhase(() -> {
                    desiredDisplacement = new VectorF(0, fwdDst + initialOffset, 0, 0);
                    setArmStage(0);
                }, EndMovementPhaseStep, MovementPhaseCheck);
                int finalI = i;
                addPhase(() -> {
                    MAX_SPEED_MULT = INITIAL_MAX_SPEED_MULT;
                    MAX_ACCEL_TIME = INITIAL_MAX_ACCEL_TIME;
                    colorDetectionPipeline.setTargetColor(coneColor);
                    colorDetectionPipeline.setRowHeight(0.6, 0.7);
                    setTargetRotation(RIGHT_ANGLE * Math.signum(leftDst));
                    setClawOpen(true);
                    goalArmEncoderValue = -275 - (reps - finalI - 1) * 100;
                }, () -> {}, () -> RotationPhaseCheck.get() && Math.abs(goalArmEncoderValue - realArmEncoderValue) < 40);
                // align with cones
                addPhase(() -> {}, () -> {
                    double dir = colorDetectionPipeline.getColorDir();
                    double movement = Math.max(Math.abs(dir * 0.5), 0.05) * Math.signum(dir);
                    double[] targetColor = colorDetectionPipeline.targetColor;
                    telemetry.addData("Pole Dir", dir);
                    telemetry.addData("Target Color", "%s %s %s", targetColor[0], targetColor[1], targetColor[2]);
                    telemetry.addData("High Row", colorDetectionPipeline.highRow);
                    telemetry.addData("Low Row", colorDetectionPipeline.lowRow);
                    setLocalMovementVector(
                            new VectorF((float) movement, 0, 0, 0)
                    );
                    // "crash" protection
                    if (displacement.subtracted(desiredDisplacement).magnitude() > 10 * IN_TO_MM) {
                        goToPhase("park");
                    }
                }, () -> Math.abs(colorDetectionPipeline.getColorDir()) < 0.05 && runtime.seconds() - phaseStartTime > 0.5 || runtime.seconds() - phaseStartTime > 5);
                addPhase(() -> {
                    double displacementX = displacement.get(0);
                    setCurrentDisplacementAs(new VectorF((float) displacementX, (float) fwdDst + initialOffset, 0, 0));
                    setLocalMovementVector(new VectorF(0,0,0,0));
                    desiredDisplacement = new VectorF((float) (-(TILE_SIZE + 5.0 * IN_TO_MM) * Math.signum(leftDst)), fwdDst + initialOffset, 0, 0);
                    setClawOpen(true);
                }, MovementPhaseStep, MovementPhaseCheck);
                addPhase(() -> {
                    setClawOpen(false);
                }, () -> {}, () -> runtime.seconds() - phaseStartTime > 0.55);
                addPhase(() -> {
                    setArmStage(1);
                }, () -> {}, () -> runtime.seconds() - phaseStartTime > 0.40);
                addPhase(() -> {
                    desiredDisplacement = new VectorF(0, fwdDst + initialOffset, 0, 0);
                }, MovementPhaseStep, () -> MovementPhaseCheck.get() && runtime.seconds() - phaseStartTime > 0.2);
                addPhase(() -> {
                    goalArmEncoderValue = -225;
                    setTargetRotation(0.0);
                }, () -> {}, RotationPhaseCheck);
            }
        }

        // go to zone
        addPhase(() -> {
            MAX_SPEED_MULT = INITIAL_MAX_SPEED_MULT;
            MAX_ACCEL_TIME = INITIAL_MAX_ACCEL_TIME;
            setArmStage(0);
            setTargetRotation(0.0);
            float zoneDst = (float) (zone == 1 ? -Math.abs(leftDst) * 1.075 : zone == 2 ? 0.0 : Math.abs(leftDst) * 1.075);
            desiredDisplacement =  new VectorF(zoneDst, fwdDst + initialOffset, 0, 0);
        }, MovementPhaseStep, () -> false, "park");

    }

}

