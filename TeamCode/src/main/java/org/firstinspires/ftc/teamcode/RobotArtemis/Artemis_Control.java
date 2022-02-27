/*
 Copyright 2021 FIRST Tech Challenge Team 9128

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.RobotArtemis;
// Code based on http://team9960.org/mecanum-drive-prototype-1-manual-drive-software/

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

// This program provides 2 gamepad control for the robot
// GAMEPAD1 Controls the wheels
//          left joystick front and back movement
//          right joystick controls goes direction (spinning)
//          controls the elevator, boom, and claw
// GAMEPAD2 Controls the elevator, claw, and boom
//          DPAD Up/Down - elevator
//          left and right bumbers - claw
//          x and y - boom in and out

// Useful Link for Servos:  https://stemrobotics.cs.pdx.edu/node/4742
// OpMode Class definition: http://ftckey.com/apis/ftc/com/qualcomm/robotcore/eventloop/opmode/OpMode.html


// Opmode is manual

@TeleOp(name = "Artemis_Control", group = "Linear Opmode")
// @Disabled

public class Artemis_Control extends LinearOpMode { // Wheels Start


    // int BOOM_FORWARD_JUMP = 500;       // positional increment going forward
    // int BOOM_REVERSE_JUMP = 500;      // positional increment going in reverse
    /* Declare OpMode members. */
    private final ElapsedTime runtime = new ElapsedTime();
    // Declaring our opMode members
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor carouselMotor = null;
    DcMotor boomMotor = null;
    Servo ClawServo = null;
    Servo ElevatorServo = null;
    //  Servo SmartServo1;
    //   DcMotor FlyWheel;
    // Smart Servo Settings
    //double LEVERUP = -.25;
    //double LEVERDOWN = .25;
    //double LEVEROFF = .5;
    Servo armServo = null;
    //   Constants
    double MAX_CLAW_POSITION = .75;
    double MIN_CLAW_POSITION = .25;
    double MAX_ELEVATOR_POSITION = 1;
    double MIN_ELEVATOR_POSITION = .65;
    double MIN_ARM_POSITION = .50;
    double MAX_ARM_POSITION = 1;
    int MAX_BOOM_POSITION = 1000;
    int MIN_BOOM_POSITION = 0;
    // declare motor speed variables
    double RF;                         // motor speed right front
    double LF;                         // motor speed left front
    double RR;                         // motor speed right rear
    double LR;                         // motor speed left rear
    double Event_Wheel_Right_X;        // joystick position right x
    double Event_Wheel_Right_Y;        // joystick position right y
    double Event_Wheel_Left_X;         // joystick position left x
    double Event_Wheel_Left_Y;         // joystick position left y
    double joyScale = 2.0;             // joyscale constrant / USED TO SHOW HOW MUCH POWER IS GIVEN THROUGH AN INPUT
    double motorMax = 4.0;             // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    double SERVO_JUMP = 0.1;           // positional increment for servo movement
    double clawPosition;              // current position of Claw
    double elevatorPosition;          // current position of Elevator
    double armPosition; // current position of Arm
    int boomPosition;              // current position of Boom


    public void SetWheelDirection() {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        //      boomMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    public void UseEncoder() {
        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Call the sleep timer to cause the program to wait for a certain number of seconds.

    public void GoToSleep(int SleepTime) {
        try {
            TimeUnit.SECONDS.sleep(SleepTime);
        } catch (InterruptedException e) {
            telemetry.addData("Status", "Sleep Failed.");
            telemetry.update();
        }

    }

// Set the drive motor run modes:
// "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
// Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
// the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)

// Set each of the 4 wheel powers to individual values

    public void SetWheelPower(double LeftFront, double RightFront, double LeftRear, double RightRear) {
        leftFront.setPower(LeftFront);
        rightFront.setPower(RightFront);
        leftRear.setPower(LeftRear);
        rightRear.setPower(RightRear);
    }

// Set each of the 4 wheel powers to zero so they stop

    public void StopWheels() {
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    // This code is for the carousel.
    public void SetcarouselMotorDirection() {
        carouselMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void StopcarouselMotor() {
        carouselMotor.setPower(0.0);
    }

// Sets carousel motor to run.

    public void RuncarouselMotor() {
        carouselMotor.setPower(0.5);
    }

    // This code is for the boom Motor.

    public void SetboomMotorDirection() {
        boomMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    // Sets boom motor to run.
    public void forwardboomMotor() {
        boomMotor.setPower(0.5);
    }

    // Sets boom motor to run.
    public void reverseboomMotor() {
        boomMotor.setPower(-0.5);
    }

    // stops the motor
    public void stopboomMotor() {
        boomMotor.setPower(0.0);
    }

    //@Override
    public void runOpMode() { //RunOpMode Start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        //FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
        carouselMotor = hardwareMap.get(DcMotor.class, "Brad");


        // Set the drive motor direction:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // These polarities are for the Neverest 20 motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        carouselMotor.setDirection(DcMotor.Direction.FORWARD);
        boomMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        boomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Set the initial settings for starting positions of the servo and motor
        clawPosition = MIN_CLAW_POSITION;            // set Claw to closed
        elevatorPosition = MIN_ELEVATOR_POSITION;    // set Elevator to full open
        armPosition = MIN_ARM_POSITION; // set THE ARM to lowest position


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) { // OpModeIsActive Start

            // Code Common to both controllers

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // *************************************************************************
            // GamePad 1 is the wheel gamepad
            // *************************************************************************

            // Reset speed variables
            LF = 0;
            RF = 0;
            LR = 0;
            RR = 0;

            // The following variables represent diffent events on the controller.
            // If you want to change use different buttons, see the following documentation.
            // http://ftckey.com/apis/ftc/index.html?com/qualcomm/robotcore/hardware/Gamepad.html

            // Get joystick values
            Event_Wheel_Left_Y = -gamepad1.right_stick_y * joyScale; // invert so up is positive
            Event_Wheel_Left_X = gamepad1.right_stick_x * joyScale;
            Event_Wheel_Right_Y = -gamepad1.left_stick_y * joyScale; // Event_Wheel_Left_Y is not used at present
            Event_Wheel_Right_X = gamepad1.left_stick_x * joyScale;

            // Forward/back movement
            LF += Event_Wheel_Right_Y;
            RF += Event_Wheel_Right_Y;
            LR += Event_Wheel_Right_Y;
            RR += Event_Wheel_Right_Y;

            // Side to side movement
            LF += Event_Wheel_Right_X;
            RF -= Event_Wheel_Right_X;
            LR -= Event_Wheel_Right_X;
            RR += Event_Wheel_Right_X;

            // Rotation movement
            // LF += Event_Wheel_Left_X; RF -= Event_Wheel_Left_X; LR += Event_Wheel_Left_X; RR -= Event_Wheel_Left_X;
            LF -= Event_Wheel_Left_X;
            RF += Event_Wheel_Left_X;
            LR -= Event_Wheel_Left_X;
            RR += Event_Wheel_Left_X;

            // Clip motor power values to +-motorMax
            LF = Math.max(-motorMax, Math.min(LF, motorMax));
            RF = Math.max(-motorMax, Math.min(RF, motorMax));
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));
            //carouselMotor = Math.max(-motorMax, Math.min(RR, motorMax));

            // Send values to the motors
            leftFront.setPower(LF);
            rightFront.setPower(RF);
            leftRear.setPower(LR);
            rightRear.setPower(RR);


            // *************************************************************************
            // GamePad 2 is the driver gamepad to control elevator, boom, and claw
            // *************************************************************************


            // Based on an event on the controller, increment or decrement elevator position and execute command to set it.
            if (gamepad2.a) {
                SetcarouselMotorDirection();
                RuncarouselMotor();
            } else {
                StopcarouselMotor();
            }


            // Based on an event on the controller, increment or decrement elevator position and execute command to set it.
            if (gamepad2.x) {
                SetboomMotorDirection();
                forwardboomMotor();
            } else {
                stopboomMotor();
            }
            if (gamepad2.y) {
                SetboomMotorDirection();
                reverseboomMotor();
            } else {
                stopboomMotor();
            }

            // Based on an event on the controller, increment or decrement claw position.
            if (gamepad2.left_bumper && clawPosition < MAX_CLAW_POSITION) {
                clawPosition += SERVO_JUMP;
                telemetry.addData("LeftClaw", "%.3f", clawPosition);
                ClawServo.setPosition(Range.clip(clawPosition, MIN_CLAW_POSITION, MAX_CLAW_POSITION));
            }

            if (gamepad2.right_bumper && clawPosition > MIN_CLAW_POSITION) {
                clawPosition -= SERVO_JUMP;
                telemetry.addData("RightClaw", "%.3f", clawPosition);
                ClawServo.setPosition(Range.clip(clawPosition, MIN_CLAW_POSITION, MAX_CLAW_POSITION));
            }

            if (gamepad2.right_trigger > 0 && armPosition < MAX_ARM_POSITION) {
                armPosition += SERVO_JUMP;
                telemetry.addData("Arm", "%.3f", armPosition);
                armServo.setPosition(Range.clip(armPosition, MIN_ARM_POSITION, MAX_ARM_POSITION));
            }

            if (gamepad2.left_trigger > 0 && armPosition > MIN_ARM_POSITION) {
                armPosition -= SERVO_JUMP;
                telemetry.addData("Arm", "%.3f", armPosition);
                armServo.setPosition(Range.clip(armPosition, MIN_ARM_POSITION, MAX_ARM_POSITION));
            }

            // Based on an event on the controller, increment or decrement elevator position and execute  command to set it.
            if (gamepad2.dpad_down && elevatorPosition < MAX_ELEVATOR_POSITION) {
                elevatorPosition += SERVO_JUMP;
                ElevatorServo.setPosition(Range.clip(elevatorPosition, MIN_ELEVATOR_POSITION, MAX_ELEVATOR_POSITION));
            }

            if (gamepad2.dpad_up && elevatorPosition > MIN_ELEVATOR_POSITION) {
                elevatorPosition -= SERVO_JUMP;
                telemetry.addData("RightElev", "%.3f", elevatorPosition);
                ElevatorServo.setPosition(Range.clip(elevatorPosition, MIN_ELEVATOR_POSITION, MAX_ELEVATOR_POSITION));
            }
        } // OpModeIsActive End
    } // RunOpMode End
} // Wheels End
