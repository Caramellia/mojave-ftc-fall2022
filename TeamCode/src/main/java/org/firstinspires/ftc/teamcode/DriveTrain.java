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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// linear algebra libraries
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.io.*;
import java.util.*;

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
@TeleOp(name="Drive Train", group="Linear Opmode")
public class DriveTrain extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();



    // distance sensor stuff
    private Rev2mDistanceSensor distanceSensorR = null;
    private Rev2mDistanceSensor distanceSensorL = null;
    private double distanceBetweenSensors = 10 * 25.4; // inches to millimeters
    private final double WALL_CONSIDERATION_THRESHOLD = Math.toRadians(2.5); // radians duh

    // wheel stuff
    private DcMotor[] wheelMap; // list of the wheel DcMotors
    private double[] wheelPowers = {0, 0, 0, 0}; // the final powers that are applied to the wheels
    private double[] axialMovementMap = {-1, -1, -1, -1}; // base wheel powers required for axial (i.e. forward/backward) movement
    private double[] lateralMovementMap = {-1, 1, 1, -1}; // base wheel powers required for lateral (i.e. side to side) movement
    private double[] turnMap = {1, 1, -1, -1}; // base wheel powers required for turning
    private double encoderTicksPerRevolution = 537.7;
    private double wheelDiameter = 100.0; // millimeters
    private double wheelDistancePerEncoderTick = (100.0 * 2.0 * Math.PI)/encoderTicksPerRevolution;

    // movement stuff
    private VectorF movementVector = new VectorF(0, 0, 0, 1); // movement vector is in the bot's local space
    private double freeMoveSpeed = 0.25; // multiplier for strafing speeds in "free movement" mode
    private double freeTurnSpeed = 0.25; // multiplier for turning speeds in "free movement" mode
    private double masterPower = 0.9; // master multiplier of wheel powers

    // rotation stuff
    private final double RIGHT_ANGLE = Math.PI/2.0;
    private BNO055IMU imu;
    private OpenGLMatrix rotationMatrix = OpenGLMatrix.identityMatrix();
    private OpenGLMatrix targetRotationMatrix = OpenGLMatrix.identityMatrix();
    private double referenceRotation = 0.0; // frame of reference rotation, used to ensure that right-angle increments are aligned with the field
    private double rotation = 0.0; // rotation of the bot as compared to the reference rotation
    private double targetRotation = 0.0; // target for "rotation" variable, achieved by turning the bot
    private double rotationDampeningThreshold = RIGHT_ANGLE * (60.0/90.0); // threshold before the motors begin to lessen their power
    private double rotationPower = 0.9; // multiplier for rotation speed
    private double turnVelocity = 0.0; //

    private boolean lastLBumper = false;
    private boolean lastRBumper = false;

    private double lastTick = 0.0;
    private double deltaTime = 0.0;

    // generic math functions
    private double modulo(double a, double b) {
        return  (a % b + b) % b;
    }

    private double normalizeAngle(double angle, AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.DEGREES) {
            return modulo((angle + 180), 360) - 180;
        } else {
            return modulo((angle + Math.PI), 2 * Math.PI) - Math.PI;
        }
    }

    private double lerp(double a, double b, double alpha) {
        return a + (b - a) * alpha;
    }

    private double roundToNearest(double num, double interval) {
        return (Math.round(num/interval) * interval);
    }

    // yeah bro
    private void applyMovement() {
        float axialMovement = movementVector.get(1);
        float lateralMovement = movementVector.get(0);
        double maxPowerMagnitude = 1;
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = (axialMovementMap[i] * axialMovement + lateralMovementMap[i] * lateralMovement + turnMap[i] * turnVelocity);
            maxPowerMagnitude = Math.max(maxPowerMagnitude, wheelPowers[i]);
        }
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = wheelPowers[i]/maxPowerMagnitude;
            wheelMap[i].setPower(wheelPowers[i] * masterPower);
        }
    }

    // rotation
    
    private void updateRotationData() {
        Orientation rawOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        rotation = normalizeAngle(rawOrientation.firstAngle - referenceRotation, AngleUnit.RADIANS);
        Orientation currentOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) rotation, 0, 0, 0);
        rotationMatrix = currentOrientation.getRotationMatrix();
    }

    private void setTurnVelocity(double vel) {
        turnVelocity = vel;
        applyMovement();
    }

    private void applyTargetRotation() {
        float turnDiff = (float) normalizeAngle(targetRotation - rotation, AngleUnit.RADIANS);
        telemetry.addData("turn diff", turnDiff);
        setTurnVelocity((float) (Math.max(Math.min(turnDiff, rotationDampeningThreshold), -rotationDampeningThreshold)/rotationDampeningThreshold * rotationPower));
    }

    private void setReferenceRotation(double val) {
        targetRotation = targetRotation - referenceRotation;
        referenceRotation = val;
        setTargetRotation(referenceRotation + targetRotation, false);
        updateRotationData();
    }

    private void setTargetRotation(double target, boolean apply) {
        targetRotation = target;
        targetRotation = normalizeAngle(targetRotation, AngleUnit.RADIANS);
        Orientation targetOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) targetRotation, 0, 0, 0);
        targetRotationMatrix = targetOrientation.getRotationMatrix();
        if (apply) {
            applyTargetRotation();
        }
    }
    

    // movement/strafing
    private void setWorldMovementVector(VectorF vector) {
        movementVector = rotationMatrix.multiplied(vector);
        applyMovement();
    }

    private void setMovementVectorRelativeToTargetOrientation(VectorF vector) {
        vector = targetRotationMatrix.inverted().multiplied(vector);
        setWorldMovementVector(vector);
    }

    private void setLocalMovementVector(VectorF vector) {
        movementVector = vector;
        applyMovement();
    }

    private VectorF getWorldMovementVector() {
        return new VectorF(0, 0, 0, 1);
    }

    private VectorF getLocalMovementVector() {
        return movementVector;
    }

    // I stole this code from some random blog post and I'm going to edit it to make it work :)
    /*
    private VectorF getRealMovementDelta() {

        //Compute change in encoder positions
        delt_m0 = wheel0Pos - lastM0;
        delt_m1 = wheel1Pos - lastM1;
        delt_m2 = wheel2Pos - lastM2;
        delt_m3 = wheel3Pos - lastM3;

        //Compute displacements for each wheel
        displ_m0 = delt_m0 * wheelDisplacePerEncoderCount;
        displ_m1 = delt_m1 * wheelDisplacePerEncoderCount;
        displ_m2 = delt_m2 * wheelDisplacePerEncoderCount;
        displ_m3 = delt_m3 * wheelDisplacePerEncoderCount;

        //Compute the average displacement in order to untangle rotation from displacement
        displ_average = (displ_m0 + displ_m1 + displ_m2 + displ_m3) / 4.0;

        //Compute the component of the wheel displacements that yield robot displacement
        dev_m0 = displ_m0 - displ_average;
        dev_m1 = displ_m1 - displ_average;
        dev_m2 = displ_m2 - displ_average;
        dev_m3 = displ_m3 - displ_average;

        //Compute the displacement of the holonomic drive, in robot reference frame
        delt_Xr = (dev_m0 + dev_m1 - dev_m2 - dev_m3) / twoSqrtTwo;
        delt_Yr = (dev_m0 - dev_m1 - dev_m2 + dev_m3) / twoSqrtTwo;

        //Move this holonomic displacement from robot to field frame of reference
        robotTheta = IMU_ThetaRAD;
        sinTheta = sin(robotTheta);
        cosTheta = cos(robotTheta);
        delt_Xf = delt_Xr * cosTheta - delt_Yr * sinTheta;
        delt_Yf = delt_Yr * cosTheta + delt_Xr * sinTheta;

        //Update the position
        X = lastX + delt_Xf;
        Y = lastY + delt_Yf;
        Theta = robotTheta;
        lastM0 = wheel0Pos;
        lastM1 = wheel1Pos;
        lastM2 = wheel2Pos;
        lastM3 = wheel3Pos;
    }
    */

    @Override
    public void runOpMode() {

        // Update control hub firmware to Firmware version 1.8.2

        // DISTANCE SENSOR SETUP
        {
            distanceSensorR = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceR");
            distanceSensorL = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceL");
        }

        // WHEEL SETUP
        {
            wheelMap = new DcMotor[]{
                    hardwareMap.get(DcMotor.class, "TopRightWheel"),
                    hardwareMap.get(DcMotor.class, "BottomRightWheel"),
                    hardwareMap.get(DcMotor.class, "TopLeftWheel"),
                    hardwareMap.get(DcMotor.class, "BottomLeftWheel")
            };

            for (int i = 0; i < 4; i++) {
                DcMotor motor = wheelMap[i];
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            wheelMap[0].setDirection(DcMotor.Direction.FORWARD);
            wheelMap[1].setDirection(DcMotor.Direction.FORWARD);
            wheelMap[2].setDirection(DcMotor.Direction.REVERSE);
            wheelMap[3].setDirection(DcMotor.Direction.REVERSE);
        }

        // IMU SETUP
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.mode = BNO055IMU.SensorMode.NDOF;
            // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu.initialize(parameters);
        }

        // INITIALIZATION TELEMETRY
        {
            telemetry.addData("Status", "Initialized.");
            telemetry.addData("Gyroscope Status", imu.isGyroCalibrated() ? "Calibrated." : "Not calibrated.");
            telemetry.addData("Magnetometer Status", imu.isMagnetometerCalibrated() ? "Calibrated." : "Not calibrated.");
            telemetry.addData("Accelerometer Status", imu.isAccelerometerCalibrated() ? "Calibrated." : "Not calibrated.");
            telemetry.addData("Game", "Press START to run >>");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // MAIN LOOP
        while (opModeIsActive()) {

            deltaTime = runtime.seconds() - lastTick;

            // ROTATION CALCULATIONS
            {
                updateRotationData();
                setTargetRotation(normalizeAngle(targetRotation, AngleUnit.RADIANS), false); // normalize the target rotation
            }

            // MOVEMENT HANDLING
            {
                VectorF rawMoveVector = new VectorF(gamepad1.left_stick_x, gamepad1.left_stick_y, 0, 1);
                if (gamepad1.right_trigger < 0.5) { // grid movement
                    setTargetRotation(roundToNearest(targetRotation, RIGHT_ANGLE), true); // snap target rotation to 90 degree angles
                    // turning
                    if (gamepad1.left_bumper && !lastLBumper) {
                        setTargetRotation(targetRotation + RIGHT_ANGLE, true);
                    }
                    if (gamepad1.right_bumper && !lastRBumper) {
                        setTargetRotation(targetRotation - RIGHT_ANGLE, true);
                    }
                    // application
                    applyTargetRotation();
                    setMovementVectorRelativeToTargetOrientation(rawMoveVector);
                } else { // free movement
                    // just a bunch of application this is ez
                    setLocalMovementVector(rawMoveVector.multiplied((float) freeMoveSpeed));
                    setTargetRotation(rotation, false); // it's immediately overridden but this is so that it snaps back to the nearest rotation after exiting free mode
                    setTurnVelocity(freeTurnSpeed * -gamepad1.right_stick_x);
                }
            }

            // ODOMETRY HANDLING
            {
                double distanceL = distanceSensorL.getDistance(DistanceUnit.MM);
                double distanceR = distanceSensorR.getDistance(DistanceUnit.MM);
                telemetry.addData("Distance L", distanceL);
                telemetry.addData("Distance R", distanceR);
                double angleAgainstWall = Math.atan((distanceL - distanceR) / distanceBetweenSensors); // clockwise turn == negative angle
                telemetry.addData("Angle Against Wall", Math.toDegrees(angleAgainstWall));

                if (distanceL != Rev2mDistanceSensor.distanceOutOfRange && distanceR != Rev2mDistanceSensor.distanceOutOfRange) {
                    // TEST IF THE "ROTATION" VALUE INCREASES OR DECREASES WITH CLOCKWISE TURNS!!!
                    double angleOfWall = rotation - angleAgainstWall;
                    telemetry.addData("Angle of Wall", Math.toDegrees(angleOfWall));
                    if (gamepad1.a) { // set this wall as the new frame of reference
                        setReferenceRotation(angleOfWall);
                        setTargetRotation(0, true);
                    }
                    boolean isWall = Math.abs(angleOfWall - roundToNearest(angleOfWall, RIGHT_ANGLE)) < WALL_CONSIDERATION_THRESHOLD;
                    telemetry.addData("Wall?", isWall ? "Yay" : "Nay");
                } else {
                    telemetry.addData("Wall?", "Nay");
                }
            }

            // OTHER TELEMETRY AND POST-CALCULATION STUFF
            {
                lastLBumper = gamepad1.left_bumper;
                lastRBumper = gamepad1.right_bumper;
                lastTick = runtime.seconds();
                telemetry.addData("Run Time", runtime.toString());
                telemetry.addData("Local Movement Vector", movementVector);
                telemetry.addData("Rotation", Math.toDegrees(rotation));
                telemetry.addData("Target Rotation", Math.toDegrees(targetRotation));
                telemetry.update();
            }
        }
    }}
