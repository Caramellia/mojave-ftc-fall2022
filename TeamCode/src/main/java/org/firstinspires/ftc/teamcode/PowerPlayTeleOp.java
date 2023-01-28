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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name="PowerPlay TeleOp", group="Linear Opmode")
public class PowerPlayTeleOp extends BaseController {

    private Gamepad lastGamepadState = new Gamepad();
    private Gamepad currentGamepadState = new Gamepad();
    private Gamepad lastGamepad2State = new Gamepad();
    private Gamepad currentGamepad2State = new Gamepad();
    private String CONTROL_STRING = "!!!! MOVEMENT CONTROLS: !!!!"
            + "\nLeft stick to move/strafe"
            + "\nBumpers to rotate 90 degrees"
            + "\nA to enter free movement mode"
            + "\nRight stick to rotate in free movement mode"
            + "\nY to calibrate orientation"
            + "\n"
            + "!!!! ARM CONTROLS: !!!!"
            + "\nX to open or close claw"
            + "\nRight trigger to raise arm by one stage, left trigger to lower arm by one stage"
            + "\nDpad UP to slowly raise arm, dpad DOWN to slowly lower arm";

    // movement stuff
    private boolean freeMovement = false;
    private double freeMoveSpeed = 0.25; // multiplier for strafing speeds in "free movement" mode
    private double freeTurnSpeed = 0.25; // multiplier for turning speeds in "free movement" mode

    // arm stuff
    private double goalSmoothArmEncoderValue = 0;
    private double smoothArmEncoderValue = 0;
    private double smoothArmEncoderVelocity = 0;
    private double decelThresholdEntranceVelocity = 0;
    private boolean isWithinDecelThreshold = false;
    private final double ARM_ENCODER_ACCEL = 6000; // ticks per second per second
    private final double ARM_ENCODER_TURNAROUND_ACCEL = ARM_ENCODER_ACCEL * 2;
    private final double ARM_ENCODER_MAX_VELOCITY = 3000; // ticks per second
    private final double ARM_ENCODER_DECEL_THRESHOLD = 200; // tick distance before the arm begins to decelerate
    private final double MIN_DECEL_VELOCITY = ARM_ENCODER_MAX_VELOCITY/5;
    private double dpadArmEncoderFactor = 0;

    // rotation stuff
    private final double RIGHT_ANGLE = Math.PI/2.0;
    private ColorDetectionPipeline colorDetectionPipeline;

    @Override
    public void setArmStage(int stage) {
        int lastStage = armStage;
        armStage = (int) clamp(stage, 0, armStageEncoderValues.length - 1);
        smoothArmEncoderValue = smoothArmEncoderValue + dpadArmEncoderFactor;
        dpadArmEncoderFactor = 0;
        goalSmoothArmEncoderValue = armStageEncoderValues[armStage];
        if (lastStage > 0 && stage == 0 && clawOpen) {
            setClawOpen(false);
        }
    }

    @Override
    public void runOpMode() {

        telemetry.addData("Controls", CONTROL_STRING);
        initialize();

        colorDetectionPipeline = new ColorDetectionPipeline(new Size(50, 240), 0.0, 0.1, new double[]{255, 255, 0});

        waitForStart();
        runtime.reset();

        // MAIN LOOP
        while (opModeIsActive()) {

            try {
                lastGamepadState.copy(currentGamepadState);
                currentGamepadState.copy(gamepad1);
            } catch (RobotCoreException e) {
                telemetry.addData("Status", "uh oh something went horribly wrong with the gamepad!");
            }
            try {
                lastGamepad2State.copy(currentGamepad2State);
                currentGamepad2State.copy(gamepad2);
            } catch (RobotCoreException e) {
                telemetry.addData("Status", "uh oh something went horribly wrong with the gamepad!");
            }

            telemetry.addData("Controls", CONTROL_STRING);
            baseUpdate();


            setTargetRotation(normalizeAngle(targetRotation, AngleUnit.RADIANS)); // normalize the target rotation

            // MOVEMENT HANDLING
            {
                VectorF rawMoveVector;
                double xMoveFactor = currentGamepadState.left_stick_x
                        + (currentGamepadState.dpad_right ? 1 : 0)
                        - (currentGamepadState.dpad_left ? 1 : 0);
                double yMoveFactor = currentGamepadState.left_stick_y
                        + (currentGamepadState.dpad_down ? 1 : 0)
                        - (currentGamepadState.dpad_up ? 1 : 0);
                if (Math.abs(xMoveFactor) + Math.abs(yMoveFactor) > 0.05) {
                    double magnitude = Math.sqrt(Math.pow(currentGamepadState.left_stick_x, 2) + Math.pow(currentGamepadState.left_stick_y, 2));
                    magnitude = Math.min(magnitude, 1.0);
                    double angle = Math.atan2(currentGamepadState.left_stick_y, currentGamepadState.left_stick_x);
                    double newAngle = roundToNearest(angle, RIGHT_ANGLE/2.0);
                    rawMoveVector = new VectorF((float) (Math.cos(newAngle) * magnitude), (float) (Math.sin(newAngle) * magnitude), 0, 0);
                } else {
                    rawMoveVector = new VectorF(0, 0, 0, 0);
                }
                if (currentGamepadState.a && !lastGamepadState.a) { // alternate movement style
                    freeMovement = !freeMovement;
                }
                if (!freeMovement) {
                    // turning
                    double nearestRightAngle = roundToNearest(targetRotation, RIGHT_ANGLE);
                    double targetRotationDiff = targetRotation - nearestRightAngle;
                    if (Math.abs(targetRotationDiff) < 0.01) {
                        if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                            setTargetRotation(roundToNearest(targetRotation, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                            setTargetRotation(targetRotation + RIGHT_ANGLE);
                        }
                        if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                            setTargetRotation(roundToNearest(targetRotation, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                            setTargetRotation(targetRotation - RIGHT_ANGLE);
                        }
                    } else {
                        if (normalizeAngle(targetRotation - nearestRightAngle, AngleUnit.RADIANS) < 0.0) {
                            if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                                setTargetRotation(nearestRightAngle);
                            }
                            if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                                setTargetRotation(nearestRightAngle - RIGHT_ANGLE);
                            }
                        } else {
                            if (currentGamepadState.left_bumper && !lastGamepadState.left_bumper) {
                                setTargetRotation(nearestRightAngle + RIGHT_ANGLE);
                            }
                            if (currentGamepadState.right_bumper && !lastGamepadState.right_bumper) {
                                setTargetRotation(nearestRightAngle);
                            }
                        }
                    }
                    // application
                    if (rawMoveVector.magnitude() < 0.01) {
                        applyTargetRotation();
                        setLocalMovementVector(new VectorF(0, 0, 0, 1));
                    } else {
                        //setTurnVelocity(0);
                        applyTargetRotation();
                        setMovementVectorRelativeToTargetOrientation(rawMoveVector);
                    }

                } else { // free movement
                    // just a bunch of application this is ez
                    setLocalMovementVector(rawMoveVector.multiplied((float) freeMoveSpeed));
                    setTargetRotation(rotation);
                    setTurnVelocity(freeTurnSpeed * -currentGamepadState.right_stick_x);
                }
            }

            // ODOMETRY HANDLING
            if (currentGamepadState.y && !lastGamepadState.y) { // set the current rotation as the new frame of reference
                setReferenceRotation(rotation);
                setTargetRotation(0);
            }

            // ARM HANDLING
            {
                boolean goUp = currentGamepadState.right_trigger > 0.5 && lastGamepadState.right_trigger <= 0.5
                    || currentGamepad2State.right_trigger > 0.5 && lastGamepad2State.right_trigger <= 0.5;
                boolean goDown = currentGamepadState.left_trigger > 0.5 && lastGamepadState.left_trigger <= 0.5
                        || currentGamepad2State.left_trigger > 0.5 && lastGamepad2State.left_trigger <= 0.5;
                if (goUp) {
                    setArmStage(armStage + 1);
                }
                if (goDown) {
                    setArmStage(armStage - 1);
                }

                double lastSmoothArmEncoderValue = smoothArmEncoderValue;
                boolean lastIsWithinDecelThreshold = isWithinDecelThreshold;
                isWithinDecelThreshold = Math.abs(smoothArmEncoderValue - goalSmoothArmEncoderValue) <= ARM_ENCODER_DECEL_THRESHOLD // if we're within the threshold...
                    && Math.signum(smoothArmEncoderVelocity) * Math.signum(goalSmoothArmEncoderValue - smoothArmEncoderValue) > 0 // AND moving toward the goal...
                    && Math.abs(smoothArmEncoderVelocity) > MIN_DECEL_VELOCITY; // AND above the min decel velocity
                if (isWithinDecelThreshold
                    && !lastIsWithinDecelThreshold) {
                    decelThresholdEntranceVelocity = smoothArmEncoderVelocity;
                }
                if (isWithinDecelThreshold) { // if we need to decelerate
                    smoothArmEncoderVelocity = Math.abs(decelThresholdEntranceVelocity) * map(
                            Math.abs(smoothArmEncoderValue - goalSmoothArmEncoderValue),
                            ARM_ENCODER_DECEL_THRESHOLD, 0,
                            1, 0,
                            true);
                    smoothArmEncoderVelocity = clamp(smoothArmEncoderVelocity, MIN_DECEL_VELOCITY, ARM_ENCODER_MAX_VELOCITY) // ensures that we're not going tooooo slow
                            * Math.signum(goalSmoothArmEncoderValue - smoothArmEncoderValue);
                } else { // otherwise we need to accelerate
                    double accelDir = Math.signum(goalSmoothArmEncoderValue - smoothArmEncoderValue); // sign effectively normalizes the value
                    smoothArmEncoderVelocity = smoothArmEncoderVelocity + accelDir * deltaTime * (
                            Math.abs(accelDir - Math.signum(smoothArmEncoderVelocity)) < 0.05 ? ARM_ENCODER_ACCEL : ARM_ENCODER_TURNAROUND_ACCEL
                    );
                    smoothArmEncoderVelocity = clamp(smoothArmEncoderVelocity, -ARM_ENCODER_MAX_VELOCITY, ARM_ENCODER_MAX_VELOCITY);
                }
                smoothArmEncoderValue += smoothArmEncoderVelocity * deltaTime;
                // if we're already at the goal point
                if ((smoothArmEncoderValue - goalSmoothArmEncoderValue) * (lastSmoothArmEncoderValue - goalSmoothArmEncoderValue) <= 0
                        || smoothArmEncoderValue == goalSmoothArmEncoderValue) {
                    // then bring it to a halt
                    smoothArmEncoderValue = goalSmoothArmEncoderValue;
                    smoothArmEncoderVelocity = 0;
                }

                double armDelta = (currentGamepadState.dpad_up ? 1 : 0) - (currentGamepadState.dpad_down ? 1 : 0)
                        + (currentGamepad2State.dpad_up ? 1 : 0) - (currentGamepad2State.dpad_down ? 1 : 0);
                dpadArmEncoderFactor -= armDelta * 750.0 * deltaTime;
                double finalValue = clamp(smoothArmEncoderValue + dpadArmEncoderFactor, 0, ARM_MIN_ENCODER_VALUE);
                dpadArmEncoderFactor = finalValue - smoothArmEncoderValue; // calculate the difference now that the final encoder value is clamped
                goalArmEncoderValue = (int) finalValue;
            }

            // CLAW HANDLING
            {
                if (currentGamepadState.x && !lastGamepadState.x) {
                    setClawOpen(!clawOpen);
                }
            }

            double poleDir = colorDetectionPipeline.getColorDir();
            telemetry.addData("Pole Dir", poleDir);

            // OTHER TELEMETRY AND POST-CALCULATION STUFF
            {
                applyMovement();
                telemetry.addData("Current Gamepad", currentGamepadState.toString());
                telemetry.addData("Last Gamepad", lastGamepadState.toString());
                telemetry.update();
            }
        }
    }}
