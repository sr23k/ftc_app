package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import java.lang.Math;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FinalTeleOp", group="Drive Opmodes")
public class FinalTeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor lift = null;
    private Servo left = null;
    private Servo right = null;

    boolean control = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack  = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");
        lift = hardwareMap.get(DcMotor.class, "lift");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        left.setDirection(Servo.Direction.REVERSE);
        right.setDirection(Servo.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setPosition(0.2);
        right.setPosition(0.2);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double FLPower = 0;
        double FRPower = 0;
        double BLPower = 0;
        double BRPower = 0;
        double speed = 0.3;
        double liftpower = 0.9;
        int threshold = 20; //joystick dead zones

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        //Mecanum Drive Mode uses left stick to strafe and go forwards and backwards; right stick to rotate
        if(Math.abs(gamepad1.left_stick_y) != 0 || Math.abs(gamepad1.left_stick_x) != 0){
            FRPower = (gamepad1.left_stick_y + gamepad1.left_stick_x);
            FLPower = (gamepad1.left_stick_y - gamepad1.left_stick_x);
            BRPower = (gamepad1.left_stick_y - gamepad1.left_stick_x);
            BLPower = (gamepad1.left_stick_y + gamepad1.left_stick_x);
        }

        if(Math.abs(gamepad1.right_stick_x) != 0){
            FRPower = (gamepad1.right_stick_x);
            FLPower = (-gamepad1.right_stick_x);
            BRPower = (gamepad1.right_stick_x);
            BLPower = (-gamepad1.right_stick_x);
        }

        if(gamepad2.a){
            left.setPosition(0.85);
            right.setPosition(0.85);
        }

        if(gamepad2.b) {
            left.setPosition(0.2);
            right.setPosition(0.2);
        }
          //controls the lift motor
          
       if(Math.abs(gamepad2.left_stick_x) != 0){
       
       lift.setPower(((0.9 - (-0.9)) * (Range.clip(gamepad2.left_stick_x,-1.0,1.0) - (-1.0)) / (1.0 - (-1.0))) + (-0.9));

       }else{
       lift.setPower(0.0);
       }
          
       /* if(gamepad2.left_bumper){
            lift.setPower(liftpower);

        }else if(gamepad2.right_bumper){
            lift.setPower(-liftpower);

        }else{
            lift.setPower(0);
        }*/

        //Toggle between coarse and fine control modes
       /* if(gamepad1.a){
            control = !control; //true equals fine control

        }
        if(control){
            speed = 0.1;
        }else{
            speed = 0.3;
        }*/
        
        double FRPower2 = Range.clip(FRPower,-1.0,1.0);
        double FLPower2 = Range.clip(FLPower,-1.0,1.0);
        double BLPower2 = Range.clip(BLPower,-1.0,1.0);
        double BRPower2 = Range.clip(BRPower,-1.0,1.0);
        
        
         //Scale controller output power to acceptable motor power range
         double FRPower3 = (((0.2 - -0.2) * (FRPower - (-1.0)) / (1 - (-1))) + (-0.2));
         double FLPower3 = (((0.2 - -0.2) * (FLPower - (-1.0)) / (1 - (-1))) + (-0.2));
         double BLPower3 = (((0.2 - -0.2) * (BLPower - (-1.0)) / (1 - (-1))) + (-0.2));
         double BRPower3 = (((0.2 - -0.2) * (BRPower - (-1.0)) / (1 - (-1))) + (-0.2));
         
        // Send calculated power to wheels
        leftFront.setPower(FLPower3);
        rightFront.setPower(FRPower3);
        leftBack.setPower(BLPower3);
        rightBack.setPower(BRPower3);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", FLPower3, FRPower3,BLPower3,BRPower3);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
