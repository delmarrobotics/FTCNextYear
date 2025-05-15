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

package test.code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import common.Logger;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Calibrate Encoder", group="Test")
@Disabled
@SuppressWarnings("unused")

public class CalibrateEncoder extends LinearOpMode {

    /* Declare OpMode members. */
    private final ElapsedTime     runtime = new ElapsedTime();

    private DcMotor motor   = null;  //  Used to control the left front drive wheel
    private int encoderCount = 0;
    private int lastCount = 0;
    private double speed = 0.2;

    private Telemetry.Item startMsg;
    private Telemetry.Item encoderMsg;
    private Telemetry.Item positionMsg;
    private Telemetry.Item speedMsg;
    private Telemetry.Item runningToMsg;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;      // eg: HD Hex Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 40 ;      // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = (96 / 25.4);     // Circumference of 96 mm wheel in inches

    @Override
    public void runOpMode() {
        telemetry.addLine("Press start");
        telemetry.update();

        initMotor();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.setAutoClear(false);
        startMsg = telemetry.addData("Encoder Calibration\nControls", "\n" +
                "  left stick - manual motor control\n" +
                "  left trigger - decrease encoder count\n" +
                "  right trigger - increase encoder count\n" +
                "  a - run motor for encoder count\n" +
                "  x - zero encode count\n" +
                "  b - set encoder count \n\n");

        encoderMsg = telemetry.addData("Encoder count", 0);
        positionMsg = telemetry.addData("Encoder position", 0);
        speedMsg = telemetry.addData("Motor speed", 0);
        runningToMsg = telemetry.addData("Running to", 0);

        encoderMsg.setValue(" %d ", encoderCount);
        speedMsg.setValue(" %4.2f", speed);
        positionMsg.setValue( "%7d", motor.getCurrentPosition());

        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // Run motor to an encoder count
                //motor.setDirection(DcMotorSimple.Direction.FORWARD);
                runToPosition(speed, encoderCount, 5.0);  // 5 second timeout

            } else if (gamepad1.x) {
                // zero calibration count
                encoderCount = 0;
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                positionMsg.setValue( "%7d", motor.getCurrentPosition());
                lastCount = motor.getCurrentPosition();
                encoderMsg.setValue("%d", encoderCount);

            } else if (gamepad1.b) {
                // set calibration count
                encoderCount = motor.getCurrentPosition();
                encoderMsg.setValue("%d", encoderCount);

            } else if (gamepad1.right_trigger > 0) {
                // increase calibration count
                runtime.reset();
                while (gamepad1.right_trigger > 0) {
                    encoderCount += increment(1, 10, 100);
                    encoderMsg.setValue("%d", encoderCount);
                    telemetry.update();
                }

            } else if (gamepad1.right_trigger > 0) {
                // decrease calibration count
                runtime.reset();
                while (gamepad1.right_trigger > 0) {
                    encoderCount -= increment(1, 10, 100);
                    encoderMsg.setValue("%d", encoderCount);
                    telemetry.update();
                }

            } else if (gamepad1.left_bumper) {
                // decrease the speed
                runtime.reset();
                while (gamepad1.left_bumper){
                    speed -= (double)increment(1, 5, 10) / 100;
                    speedMsg.setValue(" %4.2f", speed);
                    telemetry.update();
                }

            } else if (gamepad1.right_bumper) {
                // increase the speed
                runtime.reset();
                while (gamepad1.right_bumper){
                    speed += (double)increment(1, 5, 10) / 100;
                    speedMsg.setValue(" %4.2f", speed);
                    telemetry.update();
                }

            } else if (gamepad1.left_stick_y > 0) {
                // manually run the motor forward
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(-speed);
                while (true) {
                    //Logger.message("y stick %4.2f", gamepad1.left_stick_y );
                    if (gamepad1.left_stick_y <= 0)
                        break;
                }
                motor.setPower(0);
                sleep(200);
                positionMsg.setValue( "%7d", motor.getCurrentPosition());

            } else if (gamepad1.left_stick_y < 0) {
                // manually run the motor forward
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setPower(speed);
                while (true) { if (gamepad1.left_stick_y >= 0) break; }
                motor.setPower(0);
                sleep(200);
                positionMsg.setValue( "%7d", motor.getCurrentPosition());

            } else if (gamepad1.dpad_up) {
                while (gamepad1.dpad_up) {
                    Logger.message("%d %d", motor.getCurrentPosition(), motor.getTargetPosition());
                    runToPosition(0.1, motor.getTargetPosition(), 10);
                }
            }

            positionMsg.setValue( "%7d", motor.getCurrentPosition());
            telemetry.update();
        }
    }


    /**
     * Initial the motor to calibrate.
     */
    public void initMotor (){
        motor  = hardwareMap.get(DcMotor.class, "pixelElbow");
        //2982 in extended and 0 at default
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     *  Run the motor using the encoder to the specified position.
     *
     * @param speed motor power (0 - 1)
     * @param count encoder counts to move the motor
     * @param timeoutS seconds to run before timing out
     */
    public void runToPosition(double speed, int count, double timeoutS) {
        int newPosition;
        int currentPosition;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            currentPosition = motor.getCurrentPosition();
//            newPosition = currentPosition + count;
            newPosition = count;
            motor.setTargetPosition(newPosition);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(speed));

            // Send telemetry message to indicate successful Encoder reset
            runningToMsg.setValue(" %d", newPosition);
            telemetry.update();

            runtime.reset();

            while (opModeIsActive() && motor.isBusy()) {
                if (runtime.seconds() >= timeoutS){
                    Logger.message("encoderDrive timed out");
                    break;
                }
                positionMsg.setValue( "%d", motor.getCurrentPosition());
                //telemetry.addData("Running to",  " %7d, currently at %7d", newPosition, motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor.setPower(0);

            // Turn off RUN_TO_POSITION
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            positionMsg.setValue( "%d", motor.getCurrentPosition());

            //sleep(250);   // optional pause after each move.
        }
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    public int increment(int v1, int v2, int v3){
        int sleepTime;
        int delta;
        if (runtime.seconds() < 3){
            delta = v1;
            sleepTime = 500;
        }
        else if (runtime.seconds() < 6){
            delta = v2;
            sleepTime = 200;
        }
        else{
            delta = v3;
            sleepTime = 100;
        }
        sleep(sleepTime);
        return delta;
    }
}

