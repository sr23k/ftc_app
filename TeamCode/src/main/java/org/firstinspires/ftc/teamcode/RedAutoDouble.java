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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

@Autonomous(name="Auto Red Double", group="Linear Opmode")
public class RedAutoDouble extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

        // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift1 = null;
    private Servo left = null;
    private Servo right = null;

    double FLPower = 0;
    double FRPower = 0;
    double BLPower = 0;
    double BRPower = 0;
    double liftpower = 0.9;

    final double countsPerRev = 537.6;
    final int countsPerInch = 1000;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXDt5KP/////AAAAGWeVKR4ppkmsoKP4RcK6sthe5mVGiCtzcIzjeNoMcU9DeY+UJpjzOy7Imjj4NGFNf5tL78lK8cOIbNxiSZgfRpILVKwHXIvpsB3FoEb1Bsi2eg2uc2bgkwi4Ms+aCDExZbH/ltzjQJab44d07kPYMCqkfnjDnWzMugWnbXZFbvARgBbn+T3zZUMsUIpspNKSM6h9zIQrQ4kzkZVzRX/mvB8dBp4VBeIKNfjjCIIgPatNI8erwY563jPC2CgIU45TWXFeFw2Crkt2e12JUE5LGTzL0JT7jktTTpTAuSuRuDjFqdNOW2eYdl37hL8JbR2fxUND1fDP3aAKedC9tepHw7pdo5ruqdeEVsYJFUGvvEXj";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack  = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        lift1 = hardwareMap.get(DcMotor.class, "lift");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

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


        resetEncoders();

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPosition(0.2);
        right.setPosition(0.2);

        boolean LEDOn = true;

        //color.enableLed(LEDOn);

        boolean scan = true;

        String vuMarkString = "";

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            while(scan){
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                    telemetry.addData("VuMark", "%s visible", vuMark);

                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        vuMarkString = "left";
                    }

                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        vuMarkString = "right";
                    }

                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        vuMarkString = "center";
                    }
                    scan = false;
                }
            }
            //Start movement
            backward(0.1);

            //Hit Jewel
            scan = true;
            /*while(scan){
                if(isRed()){
                    scan = false;
                }else if(isBlue()){
                    scan = false;
                }
            }*/

            forward(1);
            rotate(-90);
            forward(1);
            rotate(-90);

            if (vuMarkString.equals("left")) {
                left(0.1);
                forward(0.1);
            }

            if (vuMarkString.equals("right")) {
                right(0.1);
                forward(0.1);
            }

            if (vuMarkString.equals("center")) {
                forward(0.1);
            }


            backward(1);
            rotate(180);
            //Go to glyph pile
            open();
            forward(1);
            close(); //guess at glyph?
            lift(0.5,true);
            rotate(180);
            forward(2);


            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", FLPower, FRPower);
            telemetry.update();
        }
    }
    public void lift(double time, boolean up){
        if(up){
            lift1.setPower(liftpower);
        }else{
            lift1.setPower(-liftpower);
        }

        sleep(Math.round(time * 1000)); //set in terms of seconds
        lift1.setPower(0);
    }
    public void rotate(double degrees){
        final int countsPerDegree = 750; //Calculate for 10 degrees with protractor, then divide and hope it works
        //degrees to the left, make negative to turn right
        int countsInDegrees = (int)Math.round(degrees * countsPerDegree);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + countsInDegrees);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + countsInDegrees);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() - countsInDegrees);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() - countsInDegrees);

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
    /*public boolean isRed(){
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
    /*public boolean isBlue(){
        double threshold = 2;
        boolean b = false;
        if(color.blue() > threshold && color.red() < threshold){
            telemetry.addData("Colors", "red , blue ", color.blue(), color.red());
            return true;
        }else{
            telemetry.addData("Colors", "red , blue ", color.blue(), color.red());
            return false;
        }
    }*/
    public void resetEncoders(){
        while(rightBack.getCurrentPosition() != 0 || leftFront.getCurrentPosition() != 0 || leftBack.getCurrentPosition() != 0){
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void open(){
        left.setPosition(0.85);
        right.setPosition(0.85);
    }
    public void close(){
        left.setPosition(0.2);
        right.setPosition(0.2);
    }
}
