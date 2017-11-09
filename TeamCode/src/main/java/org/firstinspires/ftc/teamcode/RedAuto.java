/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.*;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

/** Shoutout to Rishi Parikh for being a true hiu */

@Autonomous(name="Auto Red", group="Linear Opmode")
public class RedAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private Servo left = null;
    private Servo right = null;
    private ColorSensor color = null;

    double FLPower = 0;
    double FRPower = 0;
    double BLPower = 0;
    double BRPower = 0;
    double liftpower = 0.9;

    final double countsPerRev = 537.6;
    final int countsPerInch = 1000;
    int robotposition = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack  = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        lift1 = hardwareMap.get(DcMotor.class, "lift");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        color = hardwareMap.get(ColorSensor.class, "color");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(lift1.getDirection() == DcMotor.Direction.FORWARD){ //lol, never did figure out the direction
            lift2.setDirection(DcMotor.Direction.REVERSE);
        }else{
            lift2.setDirection(DcMotor.Direction.FORWARD);
        }

        resetEncoders();

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPosition(0);
        right.setPosition(0);

        boolean LEDOn = true;

        color.enableLed(LEDOn);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            forward(0.1);




            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", FLPower, FRPower);
            telemetry.update();
        }
    }
    public void lift(long time, boolean up){
        if(up){
            lift1.setPower(liftpower);
            lift2.setPower(liftpower);
        }else{
            lift1.setPower(-liftpower);
            lift2.setPower(-liftpower);
        }

        sleep(time * 1000); //set in terms of seconds
        lift1.setPower(0);
        lift2.setPower(0);
    }
    public void forward(double distance){
        int distanceInCounts = (int)Math.round(distance * countsPerInch);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + distanceInCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + distanceInCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - distanceInCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() - distanceInCounts);
        telemetry.addData("Encoder Position", "lf (%.2f) , lb (%.2f) , rb (%.2f) ", leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }
    public void backward(double distance){
        int distanceInCounts = (int)Math.round(distance * countsPerInch);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() - distanceInCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() - distanceInCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + distanceInCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + distanceInCounts);
        telemetry.addData("Encoder Position", "lf (%.2f) , lb (%.2f) , rb (%.2f) ", leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }
    public void right(double distance){
        int distanceInCounts = (int)Math.round(distance * countsPerInch);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() - distanceInCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() - distanceInCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + distanceInCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + distanceInCounts);
        telemetry.addData("Encoder Position", "lf (%.2f) , lb (%.2f) , rb (%.2f) ", leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }
    public void left(double distance){
        int distanceInCounts = (int)Math.round(distance * countsPerInch);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + distanceInCounts);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + distanceInCounts);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - distanceInCounts);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() - distanceInCounts);
        telemetry.addData("Encoder Position", "lf (%.2f) , lb (%.2f) , rb (%.2f) ", leftFront.getCurrentPosition(), leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
    }
    public boolean isAllianceColor(){
        double threshold = 2;
        boolean b = false;
        if(color.blue() < threshold && color.red() > threshold){
            telemetry.addData("Colors", "red , blue ", color.blue(), color.red());
            return true;
        }else{
            telemetry.addData("Colors", "red , blue ", color.blue(), color.red());
            return false;
        }
    }
    public void resetEncoders(){
        while(rightBack.getCurrentPosition() != 0 || leftFront.getCurrentPosition() != 0 || leftBack.getCurrentPosition() != 0){
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void open(){
        left.setPosition(0.75);
        right.setPosition(0.75);
    }
    public void close(){
        left.setPosition(0);
        right.setPosition(0);
    }
}
