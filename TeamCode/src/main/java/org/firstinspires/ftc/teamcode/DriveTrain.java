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

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor topRightWheel = null;
    private DcMotor bottomRightWheel = null;
    private DcMotor topLeftWheel = null;
    private DcMotor bottomLeftWheel = null;
    private BNO055IMU imu;

    private final double RIGHT_ANGLE = Math.PI/2;

    // distance sensor stuff

    private Rev2mDistanceSensor distanceSensorR = null;
    private Rev2mDistanceSensor distanceSensorL = null;
    private double distanceBetweenSensors = 10; // inches
    private final double WALL_CONSIDERATION_THRESHOLD = Math.toRadians(2.5); // radians duh

    // wheel stuff
    private DcMotor[] wheelMap;
    private double[] wheelPowers = {0, 0, 0, 0};
    private double[] axialMovementMap = {-1, -1, -1, -1};
    private double[] lateralMovementMap = {-1, 1, 1, -1};
    private double[] turnMap = {1, 1, -1, -1};
    private double freeMoveSpeed = 0.25;
    private double freeTurnSpeed = 0.25;
    private double masterPower = 0.9;

    public OpenGLMatrix transformMatrix = OpenGLMatrix.identityMatrix();
    private VectorF movementVector = new VectorF(0, 0, 0, 1); // in the bot's local space
    private double turnVelocity = 0;

    private double targetRotation = 0;
    private OpenGLMatrix targetTransformMatrix = OpenGLMatrix.identityMatrix();
    private double referenceRotation = 0;
    private double rotation = 0;
    private double rotationDampeningThreshold = 45;
    private double rotationPower = 1;
    private boolean lastLBumper = false;
    private boolean lastRBumper = false;

    private double lastTick = 0;
    private double deltaTime = 0;

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

    // stuff that doesn't actually affect the hardware
    public void updateTransformationData() {
        Orientation rawOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        rotation = normalizeAngle(rawOrientation.firstAngle - referenceRotation, AngleUnit.RADIANS);
        Orientation currentOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) rotation, 0, 0, 0);
        transformMatrix = currentOrientation.getRotationMatrix();
    }

    // yeah bro
    private void applyMovement() {
        VectorF up = transformMatrix.getColumn(2); // x = 0, y = 1, z = 2?
        //movementVector = movementVector.subtracted(movementVector.multiplied(movementVector.dotProduct(up)));
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

    public void setTurnVelocity(double vel) {
        turnVelocity = vel;
        applyMovement();
    }

    public void applyTargetRotation() {
        float turnDiff = (float) normalizeAngle(targetRotation - rotation, AngleUnit.RADIANS);
        telemetry.addData("turn diff", turnDiff);
        setTurnVelocity((float) (Math.max(Math.min(turnDiff, rotationDampeningThreshold), -rotationDampeningThreshold)/rotationDampeningThreshold * rotationPower));
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
        targetRotation = normalizeAngle(targetRotation, AngleUnit.RADIANS);
        Orientation targetOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS, (float) targetRotation, 0, 0, 0);
        targetTransformMatrix = targetOrientation.getRotationMatrix();
        applyTargetRotation();
    }

    public void setWorldMovementVector(VectorF vector) {
        movementVector = transformMatrix.multiplied(vector);
        applyMovement();
    }

    public void setMovementVectorRelativeToTargetOrientation(VectorF vector) {
        vector = targetTransformMatrix.inverted().multiplied(vector);
        setWorldMovementVector(vector);
    }

    public void setLocalMovementVector(VectorF vector) {
        movementVector = vector;
        applyMovement();
    }

    public VectorF getWorldMovementVector() {
        return new VectorF(0, 0, 0, 1);
    }

    public VectorF getLocalMovementVector() {
        return movementVector;
    }

    public void setReferenceRotation(double val) {
        targetRotation = targetRotation - referenceRotation;
        referenceRotation = val;
        setTargetRotation(referenceRotation + targetRotation);
        updateTransformationData();
    }

    @Override
    public void runOpMode() {

        ///////////////////////
        // DISTANCE SENSOR SETUP

        distanceSensorR = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceR");
        distanceSensorL = hardwareMap.get(Rev2mDistanceSensor.class, "DistanceL");

        ///////////////////////
        // WHEEL SETUP BEGIN //
        ///////////////////////
        topRightWheel  = hardwareMap.get(DcMotor.class, "TopRightWheel");
        bottomRightWheel  = hardwareMap.get(DcMotor.class, "BottomRightWheel");
        topLeftWheel = hardwareMap.get(DcMotor.class, "TopLeftWheel");
        bottomLeftWheel = hardwareMap.get(DcMotor.class, "BottomLeftWheel");

        wheelMap = new DcMotor[]{topRightWheel, bottomRightWheel, topLeftWheel, bottomLeftWheel};

        for (int i = 0; i < 4; i++) {
            DcMotor motor = wheelMap[i];
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        topRightWheel.setDirection(DcMotor.Direction.FORWARD);
        bottomRightWheel.setDirection(DcMotor.Direction.FORWARD);
        topLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        /////////////////////
        // WHEEL SETUP END //
        ////////////////////

        /////////////////////
        // IMU SETUP BEGIN //
        /////////////////////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.mode = BNO055IMU.SensorMode.NDOF;
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        ///////////////////
        // IMU SETUP END //
        ///////////////////

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized.");
        telemetry.addData("Gyroscope Status", imu.isGyroCalibrated() ? "Calibrated." : "Not calibrated.");
        telemetry.addData("Magnetometer Status", imu.isMagnetometerCalibrated() ? "Calibrated." : "Not calibrated.");
        telemetry.addData("Accelerometer Status", imu.isAccelerometerCalibrated() ? "Calibrated." : "Not calibrated.");
        telemetry.addData("Game", "Press START to run >>");
        telemetry.update();

        waitForStart();
        runtime.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            deltaTime = runtime.seconds() - lastTick;
            lastTick = runtime.seconds();


            // ROTATION CALCULATIONS
            updateTransformationData();
            setTargetRotation(normalizeAngle(targetRotation, AngleUnit.RADIANS)); // normalize the target rotation

            // MOVEMENT HANDLING
            VectorF rawMoveVector = new VectorF(gamepad1.left_stick_x, gamepad1.left_stick_y, 0, 1);
            if (gamepad1.right_trigger < 0.5) { // grid movement
                setTargetRotation(roundToNearest(targetRotation, RIGHT_ANGLE)); // snap target rotation to 90 degree angles
                // turning
                if (gamepad1.left_bumper && !lastLBumper) {
                    setTargetRotation(targetRotation + RIGHT_ANGLE);
                }
                if (gamepad1.right_bumper && !lastRBumper) {
                    setTargetRotation(targetRotation - RIGHT_ANGLE);
                }
                // application
                applyTargetRotation();
                setMovementVectorRelativeToTargetOrientation(rawMoveVector);
            } else { // free movement
                // just a bunch of application this is ez
                setLocalMovementVector(rawMoveVector.multiplied((float) freeMoveSpeed));
                setTargetRotation(rotation); // it's immediately overridden but this is so that it snaps back to the nearest rotation after exiting free mode
                setTurnVelocity(freeTurnSpeed * -gamepad1.right_stick_x);
            }

            // ODOMETRY HANDLING

            double distanceL = distanceSensorL.getDistance(DistanceUnit.INCH);
            double distanceR = distanceSensorR.getDistance(DistanceUnit.INCH);
            double angleAgainstWall = Math.atan((distanceL - distanceR)/distanceBetweenSensors); // clockwise turn == negative angle

            if (distanceL < Rev2mDistanceSensor.distanceOutOfRange && distanceR < Rev2mDistanceSensor.distanceOutOfRange) {
                // TEST IF THE "ROTATION" VALUE INCREASES OR DECREASES WITH CLOCKWISE TURNS!!!
                double angleOfWall = rotation - angleAgainstWall;
                telemetry.addData("Angle of Wall", Math.toDegrees(angleOfWall));
                if (gamepad1.a) { // set this wall as the new frame of reference
                    setReferenceRotation(angleOfWall);
                    setTargetRotation(0);
                }
                boolean isWall = Math.abs(angleOfWall - roundToNearest(angleOfWall, RIGHT_ANGLE)) < WALL_CONSIDERATION_THRESHOLD;
                telemetry.addData("Wall?", isWall ? "Yay" : "Nay");
            }

            lastLBumper = gamepad1.left_bumper;
            lastRBumper = gamepad1.right_bumper;
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Local Movement Vector", movementVector);
            telemetry.addData("Rotation", Math.toDegrees(rotation));
            telemetry.addData("Target Rotation", Math.toDegrees(targetRotation));
            telemetry.addData("Angle Against Wall", Math.toDegrees(angleAgainstWall));
            telemetry.update();
        }
    }}
