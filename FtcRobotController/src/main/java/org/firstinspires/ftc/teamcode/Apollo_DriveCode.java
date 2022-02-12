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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 * <p>
 * <p>
 * Code based on http://team9960.org/mecanum-drive-prototype-1-manual-drive-software/
 * <p>
 * Useful Link for Servos:  https://stemrobotics.cs.pdx.edu/node/4742
 * OpMode Class definition: http://ftckey.com/apis/ftc/com/qualcomm/robotcore/eventloop/opmode/OpMode.html
 */
@TeleOp

public class Apollo_DriveCode extends LinearOpMode {

    DcMotor leftRear = null;
    DcMotor rightRear = null;

    // declare motor speed variables
    double RR;                         // motor speed right rear
    double LR;                         // motor speed left rear
    double Event_Wheel_Right_X;        // joystick position right x
    double Event_Wheel_Right_Y;        // joystick position right y
    double Event_Wheel_Left_X;         // joystick position left x
    double Event_Wheel_Left_Y;         // joystick position left y
    double joyScale = 2.0;             // joyscale constrant / USED TO SHOW HOW MUCH POWER IS GIVEN THROUGH AN INPUT
    double motorMax = 4.0;             // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    /* Declare OpMode members. */
    private Blinker the_Hub;
    private Gyroscope imu;
    private final ElapsedTime runtime = new ElapsedTime();

    public void SetWheelDirection() {
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
    }

    public void UseEncoder() {
        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public void SetWheelPower(double LeftRear, double RightRear) {
        leftRear.setPower(LeftRear);
        rightRear.setPower(RightRear);
    }

// Set each of the 4 wheel powers to zero so they stop

    public void StopWheels() {
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }


    @Override
    public void runOpMode() { //RunOpMode Start
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        the_Hub = hardwareMap.get(Blinker.class, "The Hub");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        // Drive Motor direction:
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        // Set the drive motor run modes:
        // "RUN_USING_ENCODER" causes the motor to try to run at the specified fraction of full velocity
        // Note: We were not able to make this run mode work until we switched Channel A and B encoder wiring into
        // the motor controllers. (Neverest Channel A connects to MR Channel B input, and vice versa.)
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) { // OpModeIsActive Start

            // Code Common to both controllers

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // *************************************************************************
            // GamePad 1 is the wheel gamepad
            // *************************************************************************

            // Reset speed variables
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
            LR += Event_Wheel_Right_Y;
            RR += Event_Wheel_Right_Y;

            // Side to side movement
            LR -= Event_Wheel_Right_X;
            RR += Event_Wheel_Right_X;

            // Rotation movement
            // LF += Event_Wheel_Left_X; RF -= Event_Wheel_Left_X; LR += Event_Wheel_Left_X; RR -= Event_Wheel_Left_X;
            LR -= Event_Wheel_Left_X;
            RR += Event_Wheel_Left_X;

            // Clip motor power values to +-motorMax:
            LR = Math.max(-motorMax, Math.min(LR, motorMax));
            RR = Math.max(-motorMax, Math.min(RR, motorMax));

            // Send values to the motors:
            leftRear.setPower(LR);
            rightRear.setPower(RR);

        } // OpModeIsActive End
    } // RunOpMode End
// Wheels End
}
