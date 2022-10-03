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

    private DcMotor[] wheelMap;
    private double[] wheelPowers = {0, 0, 0, 0};
    private double[] axialMovementMap = {-1, -1, -1, -1};
    private double[] lateralMovementMap = {-1, 1, 1, -1};
    private double[] turnMap = {1, 1, -1, -1};

    public OpenGLMatrix transformMatrix = OpenGLMatrix.identityMatrix();
    private VectorF movementVector = new VectorF(0, 0, 0, 1); // in the bot's local space
    private double targetRotation = 0;
    private OpenGLMatrix targetTransformMatrix = OpenGLMatrix.identityMatrix();
    private double rotation = 0;
    private double rotationDampeningThreshold = 45;
    private double rotationPower = 0.6;
    private double lastRotationTime = 0;
    private boolean lastLBumper = false;
    private boolean lastRBumper = false;

    private double modulo(double a, double b) {
        return  (a % b + b) % b;
    }

    private double normalizeAngle(double angle) {
        return modulo((angle + 180), 360) - 180;
    }

    private double Lerp(double a, double b, double alpha) {
        return a + (b - a) * alpha;
    }

    // yeah bro
    private void applyMovement() {
        VectorF up = transformMatrix.getColumn(2); // x = 0, y = 1, z = 2?
        //movementVector = movementVector.subtracted(movementVector.multiplied(movementVector.dotProduct(up)));
        float axialMovement = movementVector.get(1);
        float lateralMovement = movementVector.get(0);
        float turnDiff = (float) normalizeAngle(targetRotation - rotation);
        telemetry.addData("turn diff", turnDiff);
        float turn = (float) (Math.max(Math.min(turnDiff, rotationDampeningThreshold), -rotationDampeningThreshold)/rotationDampeningThreshold * rotationPower);
        double maxPowerMagnitude = 1;
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = (double) (axialMovementMap[i] * axialMovement + lateralMovementMap[i] * lateralMovement + turnMap[i] * turn);
            maxPowerMagnitude = Math.max(maxPowerMagnitude, wheelPowers[i]);
        }
        for (int i = 0; i < 4; i++) {
            wheelPowers[i] = wheelPowers[i]/maxPowerMagnitude;
            wheelMap[i].setPower(wheelPowers[i]);
        }
    }

    public void setTargetRotation(double target) {
        targetRotation = target;
        targetRotation = normalizeAngle(targetRotation);
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

    @Override
    public void runOpMode() {

        ///////////////////////
        // WHEEL SETUP BEGIN //
        ///////////////////////
        topRightWheel  = hardwareMap.get(DcMotor.class, "TopRightWheel");
        bottomRightWheel  = hardwareMap.get(DcMotor.class, "BottomRightWheel");
        topLeftWheel = hardwareMap.get(DcMotor.class, "TopLeftWheel");
        bottomLeftWheel = hardwareMap.get(DcMotor.class, "BottomLeftWheel");

        wheelMap = new DcMotor[]{topRightWheel, bottomRightWheel, topLeftWheel, bottomLeftWheel};

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
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        ///////////////////
        // IMU SETUP END //
        ///////////////////

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            targetRotation = normalizeAngle(targetRotation);
            Orientation currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            rotation = currentOrientation.firstAngle;

            transformMatrix = currentOrientation.getRotationMatrix();
            Orientation targetOrientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES, (float) targetRotation, 0, 0, 0);
            targetTransformMatrix = targetOrientation.getRotationMatrix();

            if (gamepad1.left_bumper == true && lastLBumper == false) {
                setTargetRotation(targetRotation + 90);
            }
            if (gamepad1.right_bumper == true && lastRBumper == false) {
                setTargetRotation(targetRotation - 90);
            }

            setMovementVectorRelativeToTargetOrientation(new VectorF(gamepad1.left_stick_x, gamepad1.left_stick_y, 0, 1));
            applyMovement();

            lastRotationTime = runtime.seconds();
            lastLBumper = gamepad1.left_bumper;
            lastRBumper = gamepad1.right_bumper;
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Local Movement Vector", movementVector);
            telemetry.addData("Orientation", currentOrientation.toString());
            telemetry.addData("Target Rotation", targetRotation);
            telemetry.update();
        }
    }}
