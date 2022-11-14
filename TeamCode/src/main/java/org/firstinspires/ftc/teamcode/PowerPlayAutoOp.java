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

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

import java.util.ArrayList;
import java.util.HashMap;

class Phase {

    public Runnable Init;
    public Runnable Step;
    public Supplier<Boolean> Check;

    public Phase(Runnable initFunc, Runnable stepFunc, Supplier<Boolean> checkFunc) {
        Init = initFunc;
        Step = stepFunc;
        Check = checkFunc;
    }

}

@Autonomous(name="PowerPlay AutoOp", group="Linear Opmode")
public class PowerPlayAutoOp extends BaseController {

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
    private VectorF desiredDisplacement = displacementVector;
    private final boolean goToNextPhase = true;
    private final double clawOpenTime = 0;

    float leftDst = (float) -TILE_SIZE;
    float fwdDst = (float) (-TILE_SIZE * 2.5);

    ArrayList<Phase> phases = new ArrayList<>();

    Supplier<Boolean> MovementPhaseCheck = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacementVector);
        return diff.magnitude() < PHASE_CHANGE_THRESHOLD;
    };

    Runnable MovementPhaseStep = () -> {
        VectorF diff = desiredDisplacement.subtracted(displacementVector);
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

    private void addPhase(Runnable init, Runnable step, Supplier<Boolean> check) {
        phases.add(new Phase(init, step, check));
    }

    // opencv
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    final float DECIMATION_LOW = 2;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    double tagsize = 0.166;
    int zone = -1;

    // dist: 62.5 inches

    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        super.initialize();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);

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

        // left distance: 25 in
        // forward distance: 62.5 in

        // zone detection phase
        addPhase(() -> {
            clawOpen = false;
            setArmStage(1);
            desiredDisplacement = new VectorF(leftDst, 0, 0, 0);
        }, () -> {
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();
            if (zone == -1 && detections != null && detections.size() > 0) {
                zone = detections.get(0).id;
            }
        }, () -> zone != -1);

        // reset phase
        addPhase(() -> desiredDisplacement = new VectorF(0, (float) (-4.0 * IN_TO_MM), 0, 0), MovementPhaseStep, MovementPhaseCheck);

        // navigation to pole phases
        float midLeftDst = (float) ((TILE_SIZE/2.0) * Math.signum(leftDst));
        addPhase(() -> {
            // reset the displacement vector because I don't feel like rewriting this code lololol
            displacementVector = new VectorF(0, 0, 0, 0);
            desiredDisplacement = new VectorF(leftDst, 0, 0, 0);
            }, MovementPhaseStep, MovementPhaseCheck);
        addPhase(() -> desiredDisplacement = new VectorF(leftDst, fwdDst, 0, 0), MovementPhaseStep, MovementPhaseCheck);
        addPhase(() -> desiredDisplacement =  new VectorF(midLeftDst, fwdDst, 0, 0), MovementPhaseStep, MovementPhaseCheck);


        // arm phases
        addPhase(() -> setArmStage(3), () -> {}, () -> Math.abs(realArmEncoderValue - goalArmEncoderValue) < 40);
        addPhase(() -> desiredDisplacement =  new VectorF(midLeftDst, fwdDst - 6.5f * (float) IN_TO_MM, 0, 0), MovementPhaseStep, () -> MovementPhaseCheck.get() && runtime.seconds() - phaseStartTime > 5.0);
        addPhase(() -> desiredDisplacement =  new VectorF(midLeftDst, fwdDst, 0, 0), MovementPhaseStep, MovementPhaseCheck);
        addPhase(() -> setArmStage(0), () -> {}, () -> Math.abs(realArmEncoderValue - goalArmEncoderValue) < 40);


        // navigation to zone phases
        addPhase(() -> desiredDisplacement = new VectorF(leftDst, fwdDst, 0, 0), MovementPhaseStep, MovementPhaseCheck);
        addPhase(() -> desiredDisplacement = new VectorF(leftDst, (float) (-TILE_SIZE), 0, 0), MovementPhaseStep, MovementPhaseCheck);
        addPhase(() -> desiredDisplacement = new VectorF((float) (zone == 1 ? -TILE_SIZE : zone == 2 ? 0.0 : TILE_SIZE), (float) (-TILE_SIZE), 0, 0), MovementPhaseStep, MovementPhaseCheck);

        setClawOpen(false);
        waitForStart();
        runtime.reset();

        // MAIN LOOP
        while (opModeIsActive()) {

            baseUpdate();
            telemetry.addData("Zone", zone);
            Phase phaseFunc = phases.get(phase);

            phaseFunc.Step.run();
            if (!phaseEndReached && phaseFunc.Check.get()) {
                phaseEndReached = true;
                phaseEndReachedTime = runtime.seconds();
                telemetry.addData("yeah", "buddy");
            }

            if (phaseEndReached && runtime.seconds() - phaseEndReachedTime > 0.5) {
                phaseEndReached = false;
                phaseStartTime = runtime.seconds();
                phase += 1;
                phases.get(phase).Init.run();
            }

            telemetry.addData("Go To Next Phase?", goToNextPhase);
            telemetry.addData("Phase End Reached?", phaseEndReached);
            telemetry.addData("Phase End Reached Time", phaseEndReachedTime);
            // OTHER TELEMETRY AND POST-CALCULATION STUFF
            {
                applyTargetRotation();
                applyMovement();
                telemetry.update();
            }
        }
    }}