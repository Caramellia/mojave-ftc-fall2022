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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* TODO:
- Add precise/slow mode
- Make rotation in 90 degree increments in fast mode
- Make an actually good acceleration integrator to calculate position based on IMU readings
- Make something to identify the bot's location using cameras
    - Use edge detection to identify borders of the images set up around the field?
- Use TensorFlow Object Detection to identify cones
*/

// android studio test
// This class operates the wheels by interfacing with the gamepad and calculates motion/location data for the drive train.
@Autonomous(name="Scrim AutoOp R", group="Linear Opmode")
public class Scrim_AutoOp_R extends BaseController {

    // rotation stuff
    private final double RIGHT_ANGLE = Math.PI/2.0;
    private int phase = 0;
    private final double MAX_SPEED_MULT = 0.6;
    private final double MAX_ACCEL_TIME = 1;
    private final double IN_TO_MM = 25.4;
    private final double SLOW_BEGIN_THRESHOLD = 12.5 * IN_TO_MM;
    private final double PHASE_CHANGE_THRESHOLD = 0.5 * IN_TO_MM;
    private final double ROTATION_PHASE_CHANGE_THRESHOLD = 2.5;
    private double phaseStartTime = 0;
    private boolean phaseChanged = false;
    private boolean movementPhase = true;
    private VectorF desiredDisplacement = displacementVector;
    private boolean goToNextPhase = true;
    private double clawOpenTime = 0;

    // dist: 62.5 inches

    @Override
    public void runOpMode() {

        initialize();
        waitForStart();
        runtime.reset();


        // left distance: 25 in
        // forward distance: 62.5 in


        // MAIN LOOP
        while (opModeIsActive()) {


            baseUpdate();
            desiredDisplacement = displacementVector;
            double leftDst = -25 * IN_TO_MM;
            double fwdDst = -62.5 * IN_TO_MM;
            if (phase == 0) {
                clawOpen = false;
                desiredDisplacement = new VectorF((float) leftDst, 0, 0, 0);
            } else if (phase == 1) {
                desiredDisplacement = new VectorF((float) leftDst, (float) fwdDst, 0, 0);
            } else if (phase == 2) {
                if (movementPhase) {
                    setTargetRotation(targetRotation - 90);
                    movementPhase = false;
                }
            } else if (phase == 3) {
                movementPhase = true;
                desiredDisplacement = new VectorF((float) leftDst, (float) (fwdDst + 25.0 * IN_TO_MM), 0, 0);
            } else if (phase == 4) {
                if (goToNextPhase) {
                    clawOpenTime = runtime.seconds();
                }
                armStage = 3;
                goToNextPhase = false;
                if (Math.abs(realArmEncoderValue - goalArmEncoderValue) < 10) {
                    clawOpenTime = runtime.seconds();
                    clawOpen = true;
                }
                if ((runtime.seconds() - clawOpenTime) > 5) {
                    goToNextPhase = true;
                }
            }
            VectorF diff = desiredDisplacement.subtracted(displacementVector);
            if (movementPhase && diff.magnitude() > 0.0) {
                double speedMult = (Math.min(runtime.seconds() - phaseStartTime, MAX_ACCEL_TIME) / MAX_ACCEL_TIME) // initial acceleration
                        * Math.max(diff.magnitude(), SLOW_BEGIN_THRESHOLD) / SLOW_BEGIN_THRESHOLD
                        * MAX_SPEED_MULT; // ending deceleration;
                VectorF dir = diff.multiplied((float) (1.0/diff.magnitude())).multiplied((float) speedMult).multiplied((float) -1.0);
                telemetry.addData("Movement Dir", dir);
                setLocalMovementVector(dir);
            } else {
                setLocalMovementVector(new VectorF(0, 0, 0, 1));
            }
            if ((movementPhase && diff.magnitude() < PHASE_CHANGE_THRESHOLD)
                    || (!movementPhase && Math.abs(targetRotation - rotation) < ROTATION_PHASE_CHANGE_THRESHOLD)) {
                if (goToNextPhase) {
                    phaseStartTime = runtime.seconds();
                    phase += 1;
                }
            }

            // OTHER TELEMETRY AND POST-CALCULATION STUFF
            {
                applyMovement();
                telemetry.update();
            }
        }
    }}
