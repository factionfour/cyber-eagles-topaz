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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This  OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android  Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop Tank", group="Robot")

public class RobotTeleopTank_Iterative extends OpMode{
// define each motor and servo
    /* Declare OpMode members. */
    public DcMotor  frontleftDrive   = null;
    public DcMotor  backleftDrive  = null;
    public DcMotor  backrightDrive  = null;
    public DcMotor  frontrightDrive = null;
    public DcMotor  armExtendo = null;
    public DcMotor  arm = null;
     //public DcMotor  leftArm     = null;
    public Servo    servoclamp1    = null;
    public Servo    servoclamp2    = null;
//    public Servo    servo2    = null;
    //public Servo    rightClaw   = null;
    int armOffset180 = 250;
    int armOffset0 = 0;
    int armOffset = 0;
    double clawOffset = 0;
    // create public static speeds for each
    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;        // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.25 ;   // Run arm motor up at 50% power
    public static final double ARM_DOWN_POWER  = -0.25 ;   // Run arm motor down at -25% power
    public static final double EXTENDO_POWER = 0.25;
    public static  final  double ARM_SPEED = 0.25;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        frontleftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        arm = hardwareMap.get(DcMotor.class, "arm_one");
        armExtendo = hardwareMap.get(DcMotor.class, "arm_extendo");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left and right sticks forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontleftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.REVERSE);
        backleftDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servoclamp1  = hardwareMap.get(Servo.class, "servo_one");
        servoclamp1.setPosition(MID_SERVO);
        servoclamp2  = hardwareMap.get(Servo.class, "servo_two");
        servoclamp2.setPosition(MID_SERVO);
        armExtendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        servo2  = hardwareMap.get(Servo.class, "servo_two");
//        servo2.setPosition(MID_SERVO);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        //rightClaw = hardwareMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);

        // Send telemetry message to signify robot waiting;
       armExtendo.setTargetPosition(0);
        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double front;
        double turn;
        double strafe;
        boolean suckin;
        boolean suckout;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
        front = gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;
        suckin = gamepad1.dpad_down;
        suckout = gamepad1.dpad_up;

        frontleftDrive.setPower(front);
        frontrightDrive.setPower(front);
        backleftDrive.setPower(front);
        backrightDrive.setPower(front);

        frontleftDrive.setPower(-turn);
        frontrightDrive.setPower(turn);
        backleftDrive.setPower(-turn);
        backrightDrive.setPower(turn);

        frontleftDrive.setPower(-strafe);
        frontrightDrive.setPower(strafe);
        backleftDrive.setPower(strafe);
        backrightDrive.setPower(-strafe);

        // changes the position to go down
        if (gamepad1.dpad_left)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.dpad_right)
            clawOffset -= CLAW_SPEED;

        servoclamp1.setPosition(clawOffset);

        servoclamp2.setPosition(-clawOffset);


//        servo2.setPosition(clawOffset);

//        if (gamepad1.b) {
//            servo1.setPosition(0.5);
//        }
//
//        if (gamepad1.a) {
//            servo1.setPosition(-1);
//        }
//
//        if (gamepad1.y) {
//            servo1.setPosition(1);
//        }
//
//        if (gamepad1.x) {
//            servo1.setPosition(-0.5);
//        }
//
//        if (gamepad1.dpad_down) {
//            armExtendo.setTargetPosition(30);
//        }

//        second servo dpad buttons
//        if (gamepad1.dpad_right) {
//            servo2.setPosition(0.5);
//        }
//
//        if (gamepad1.dpad_down) {
//            servo2.setPosition(-1);
//        }
//
//        if (gamepad1.dpad_up) {
//            servo2.setPosition(1);
//        }
//
//        if (gamepad1.dpad_left) {
//            servo2.setPosition(-0.5);
//        }


//        // Use gamepad left & right Bumpers to open and close the claw
//        if (gamepad1.right_bumper)
//            servo1.setPosition(1);//clawOffset += CLAW_SPEED;
//        else if (gamepad1.left_bumper)
//            servo1.setPosition();
//            //clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -1.0, 1.0);
//        leftClaw.setPosition(MID_SERVO + clawOffset);
//        rightClaw.setPosition(MID_SERVO - clawOffset);


//         Use gamepad buttons to move the arm up (Y) and down (A)
        //     armOffset = Range.clip(armOffset, -1.0, 1.0);

        if (gamepad1.y){
            arm.setPower(ARM_SPEED);
            arm.setTargetPosition(armOffset180);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
        else if (gamepad1.a){
            arm.setPower(ARM_SPEED);
            arm.setTargetPosition(armOffset0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);}
            else{
                arm.setPower(0.0);}

        armOffset = (int) Range.clip(armOffset, 0.0, 500.0);

        if (gamepad1.dpad_down) {
            armOffset += ARM_SPEED;
        arm.setTargetPosition(armOffset);}
        else if (gamepad1.dpad_up){
            armOffset -=ARM_SPEED ;
            arm.setTargetPosition(armOffset);}

   //     if (gamepad1.left_bumper)
  //          armOffset += ARM_SPEED;
  //      else if (gamepad1.left_trigger > 0)
 //           armOffset -= ARM_SPEED;

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("front",  "%.2f", front);
        telemetry.addData("turn", "%.2f", turn);
        telemetry.addData("arm",arm.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
