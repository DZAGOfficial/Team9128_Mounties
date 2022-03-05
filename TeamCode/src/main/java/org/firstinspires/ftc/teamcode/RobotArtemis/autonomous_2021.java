package org.firstinspires.ftc.teamcode.RobotArtemis;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;
/*
Copyright 2022 FIRST Tech Challenge Team 9128

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


        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.Blinker;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Gyroscope;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;

        import java.util.concurrent.TimeUnit;

@Autonomous

public class autonomous_2021 extends LinearOpMode {
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    DcMotor carouselMotor = null;
    DcMotor boomMotor = null;
    public Servo ClawServo = null;
    public Servo ElevatorServo = null;
    public Servo armServo = null;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        ClawServo = hardwareMap.servo.get("clawServo");
        ElevatorServo = hardwareMap.servo.get("elevatorServo");
        carouselMotor = hardwareMap.get(DcMotor.class, "Brad");
        boomMotor = hardwareMap.get(DcMotor.class, "boomMotor");
        armServo = hardwareMap.servo.get("armServo");


        double MoveSideways = 1.0;
       // double MoveForward = 1.0;

        //   Constants
        double LFPower = 0.5;
        double RFPower = 0.5;
        double LRPower = 0.5;
        double RRPower = 0.5;

// Start notifying the operator what is happening

        telemetry.addData("Status", "Ready to kick off Autonomous mode");
        telemetry.addData("Left Front", "%.3f", leftFront);
        telemetry.addData("Right Front", "%.3f", rightFront);
        telemetry.addData("Left Rear", "%.3f", leftRear);
        telemetry.addData("Right Rear", "%.3f", rightRear);
        telemetry.update();
        telemetry.update();

        // Initializing autonomous
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            while (opModeIsActive() && (runtime.seconds() < MoveSideways))
            {
                telemetry.addData("Runtime", "%.2f",runtime.seconds());
                telemetry.update();


                leftFront.setDirection(DcMotor.Direction.FORWARD);
                leftRear.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                rightRear.setDirection(DcMotor.Direction.FORWARD);

                leftFront.setPower(-LFPower);
                rightFront.setPower(-RFPower);
                leftRear.setPower(LRPower);
                rightRear.setPower(RRPower);
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            runtime.reset();

// Wait for the game to start (driver presses PLAY)
            telemetry.addData("Status", "Running");
            telemetry.update();
            runtime.reset();

        }
    }
}