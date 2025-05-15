/*
 * This OpMode calibrate any of the robot motors.
 */

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Arrays;
import java.util.Locale;
import java.util.SortedSet;

import common.Logger;
import utils.Increment;

@TeleOp(name="Calibrate Motor", group="Test")
@SuppressWarnings("unused")
@com.acmerobotics.dashboard.config.Config

public class CalibrateMotor extends LinearOpMode {

    public static double  speed = 0.25;

    private final String calibrationPath = "/temp/motorCalibration.txt";

    public enum MOTOR_SELECT { PREVIOUS, NEXT }

    private final ElapsedTime runtime = new ElapsedTime();

    private final int incrementSlow = 1;
    private final int incrementMedium = 10;
    private final int incrementFast = 100;

    private final double incrementSpeedSlow = 0.01;
    private final double incrementSpeedMedium = 0.02;
    private final double incrementSpeedFast = 0.05;

    private DcMotor motor   = null;

    private boolean holdPower = true;

    private static class MotorInfo implements Comparable<MotorInfo>{
        String      name;
        DcMotor     motor;
        int         home;
        int         target;
        double      speed;

        @Override
        public int compareTo(MotorInfo o) {
            // Method the sort the list of motors
            //return name.compareTo(o.name);
            return motor.getPortNumber() - o.motor.getPortNumber();
        }
    }
    MotorInfo[] motors = new MotorInfo[12];
    int motorCount;
    int currentMotor;

    @Override
    public void runOpMode() {

        Increment speedIncrement = new Increment(incrementSpeedSlow, incrementSpeedMedium, incrementSpeedFast);

        getMotors();
        //readCalibration();

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        waitForStart();

        Telemetry.Item motorNameMsg =  telemetry.addData("Motor name", 0);
        Telemetry.Item directionMsg = telemetry.addData("Motor direction", 0);
        Telemetry.Item brakingMsg = telemetry.addData("Motor braking", 0);
        Telemetry.Item speedMsg = telemetry.addData("Motor speed", 0);
        Telemetry.Item positionMsg = telemetry.addData("Motor position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        setDisplayName (motorNameMsg);
        setDisplayDirection(directionMsg);
        setDisplaySpeed(speedMsg);
        setDisplayBraking(brakingMsg);
        setDisplayPosition(positionMsg);
        setDisplayHome(homeMsg);
        setDisplayTarget(targetMsg);

        telemetry.addData("\nMotor Calibration Controls", "\n" +
                "  dpad left - select previous motor\n" +
                "  dpad right - select next motor\n" +
                "  dpad up - change motor direction\n" +
                "  dpad down - change motor breaking\n" +
                "  left trigger - run motor backwards\n" +
                "  right trigger - run the motor forward\n" +
                "  left stick - increase/decrease target position\n" +
                "  right stick - increase/decrease home position\n" +
                "  left bumper - decrease motor speed\n" +
                "  right bumper - increase motor speed\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  x - run to home position\n" +
                "  b - run motor to target position\n" +
                "  back - exit without saving\n" +
                "\n");

        telemetry.update();
        boolean save = true;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                motors[currentMotor].target = motor.getCurrentPosition();

            } else if (gamepad1.y) {
                // set the home position to the current position
                motors[currentMotor].home = motor.getCurrentPosition();

            } else if (gamepad1.x) {
                // run to home position
                runToPosition(motors[currentMotor].home);
                while (gamepad1.x) sleep(10);

            } else if (gamepad1.b) {
                // run motor to an target position
                runToPosition(motors[currentMotor].target);
                while (gamepad1.b) sleep(10);

            } else if (gamepad1.left_trigger > 0) {
                // manually run the motor backwards
                motor.setPower(-speed);
                while (gamepad1.left_trigger > 0) {
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    Logger.message("y %f", gamepad1.left_trigger);
                    sleep(100);
                }
                motor.setPower(0);

            } else if (gamepad1.right_trigger > 0) {
                // manually run the motor forward
                motor.setPower(speed);
                while (gamepad1.right_trigger > 0) {
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }
                motor.setPower(0);

            } else if (gamepad1.left_stick_y != 0) {
               // increase / decrease target position
               runtime.reset();
               while (gamepad1.left_stick_y != 0)  {
                   int inc = increment(incrementSlow, incrementMedium, incrementFast);
                   if (gamepad1.left_stick_y > 0)
                       motors[currentMotor].target -= inc;
                   else if (gamepad1.left_stick_y < 0)
                       motors[currentMotor].target += inc;
                   setDisplayTarget(targetMsg);
                   telemetry.update();
               }

            } else if (gamepad1.right_stick_y != 0) {
                // increase / decrease home position
                runtime.reset();
                while (gamepad1.right_stick_y != 0) {
                    int inc = increment(incrementSlow, incrementMedium, incrementFast);
                    if (gamepad1.right_stick_y > 0)
                        motors[currentMotor].home -= inc;
                    else if (gamepad1.right_stick_y < 0)
                        motors[currentMotor].home += inc;
                    setDisplayHome(homeMsg);
                    telemetry.update();
                }

            } else if (gamepad1.left_bumper) {
                // increase home position
                speedIncrement.reset();
                while (gamepad1.left_bumper) {
                    speed = Math.max(speed - speedIncrement.get(), 0);
                    setDisplaySpeed(speedMsg);
                    telemetry.update();
                }

            } else if (gamepad1.right_bumper) {
                // decrease the home position
                runtime.reset();
                while (gamepad1.right_bumper) {
                    speed = Math.min(speed + speedIncrement.get(), 0.95);
                    setDisplaySpeed(speedMsg);
                    telemetry.update();
                }
                motor.setPower(0);

            } else if (gamepad1.dpad_left) {
                // select the next motor
                selectMotor(MOTOR_SELECT.PREVIOUS);
                while (gamepad1.dpad_left){
                    sleep(10);
                }
                setDisplayName (motorNameMsg);

            } else if (gamepad1.dpad_right) {
                // select the previous motor
                selectMotor(MOTOR_SELECT.NEXT);
                while (gamepad1.dpad_right){
                    sleep(10);
                }
                setDisplayName (motorNameMsg);

            } else if (gamepad1.dpad_up) {
                // change the direction of the motor
                if (motor.getDirection() == DcMotor.Direction.FORWARD)
                    motor.setDirection(DcMotor.Direction.REVERSE);
                else
                    motor.setDirection(DcMotor.Direction.FORWARD);
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_up) {
                    sleep(10);
                }

            } else if (gamepad1.dpad_down) {
                // change motor breaking
                if (motor.getPowerFloat())
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                else
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setDisplayBraking(directionMsg);
                while (gamepad1.dpad_up) {
                    sleep(10);
                }

            } else if (gamepad1.back) {
                // exit opmode without saving calibration data
                save = false;
                requestOpModeStop();
                break;
            }

            setDisplayPosition(positionMsg);
            setDisplayHome(homeMsg);
            setDisplayTarget(targetMsg);
            telemetry.update();
        }

        if (save) {
            writeCalibration();
        }
    }

    void setDisplayName (Telemetry.Item item) {
        item.setValue("%s  (port: %d)", motors[currentMotor].name, motor.getPortNumber());
    }

    private void setDisplayPosition (Telemetry.Item item) {
        item.setValue( "%d", motors[currentMotor].motor.getCurrentPosition());
    }

    private void setDisplayHome (Telemetry.Item item) {
        item.setValue( "%d", motors[currentMotor].home);
    }

    private void setDisplayTarget (Telemetry.Item item) {
        item.setValue( "%d", motors[currentMotor].target);
    }

    private void setDisplayDirection (Telemetry.Item item) {
        item.setValue("%s", motors[currentMotor].motor.getDirection());
    }

    private void setDisplayBraking (Telemetry.Item item) {
        item.setValue("%s", motors[currentMotor].motor.getZeroPowerBehavior());
    }

    private void setDisplaySpeed (Telemetry.Item item) {
        item.setValue( "%5.3f", speed);
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    public int increment(int v1, int v2, int v3){
        int sleepTime;
        int delta;
        if (runtime.seconds() < 3) {
            delta = v1;
            sleepTime = 500;
        } else if (runtime.seconds() < 6) {
            delta = v2;
            sleepTime = 500;
        } else if (runtime.seconds() < 9) {
            delta = v3;
            sleepTime = 500;
        } else {
            delta = v3;
            sleepTime = 250;
        }

        sleep(sleepTime);
        return delta;
    }

    private void runToPosition(int position) {

        DcMotor.RunMode mode = motor.getMode();
        Logger.message("run from %d to %d", motor.getCurrentPosition(), position);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);
        while (opModeIsActive()) {
            Logger.message("position %d", motor.getCurrentPosition());
            if (! motor.isBusy())
                break;
        }
        if (holdPower) {
            runtime.reset();
            while (runtime.seconds() > 5 && opModeIsActive())
                sleep(100);
        }
        motor.setPower(0);
        motor.setMode(mode);
    }

     // Build a list of motors sorted by port number
    private void getMotors(){

        motorCount = 0;
        SortedSet<String> names = hardwareMap.getAllNames(DcMotor.class);
        for (String name: names){
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            
            motors[motorCount] = new MotorInfo();
            motors[motorCount].name = name;
            motors[motorCount].motor = m;
            motors[motorCount].home = 0;
            motors[motorCount].target = 0;
            motorCount++;
        }

        // Sort by port numbers
        Arrays.sort(motors, 0, motorCount);

        for (int i = 0; i < motors.length; i++) {
            if (motors[i] != null) {
                if (motor == null) {
                    motor = motors[i].motor;
                    currentMotor = i;
                }
                Logger.message("motor name %s port %d", motors[i].name, motors[i].motor.getPortNumber());
            }
        }
    }

    private void selectMotor (MOTOR_SELECT select){
        int index;
        for (int i = 1; i <= motors.length; i++) {
            if (select == MOTOR_SELECT.NEXT)
                index = (currentMotor + i) %  motors.length;
            else
                index = (currentMotor + motors.length - i) %  motors.length;

            if (motors[index] != null){
                motor = motors[index].motor;
                currentMotor = index;
                break;
            }
        }

    }

    // Write the calibration values to a comma separated text file.
    private void writeCalibration() {

        try {
            String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), calibrationPath);
            Logger.message ("Writing calibration data to %s", path);

            FileWriter f = new FileWriter(path);

            for (MotorInfo motorInfo : motors) {
                if (motorInfo != null) {
                    String str = String.format(Locale.ENGLISH, "%s,%d,%d\n", motorInfo.name, motorInfo.home, motorInfo.target);
                    f.write(str);
                    Logger.message(str);
                }
            }

            f.flush();
            f.close();

        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    // Read the calibration values from a comma separated text file.
    private void readCalibration() {
        try {
            String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), calibrationPath);
            Logger.message ("Reading calibration data from %s", path);

            BufferedReader reader;
            try {
                reader = new BufferedReader(new FileReader(path));
            } catch (java.io.FileNotFoundException ex) {
                Logger.message("Motor calibration file not found");
                return;
            }

            String line = reader.readLine();
            while (line != null) {
                // parse the line read
                int end = line.indexOf(',');
                if (end > 0) {
                    String name = line.substring(0, end);
                    int start = end + 1;
                    end = line.indexOf(',', start);
                    if (end > 0) {
                        String homeStr =  line.substring(start, end);
                        start = end + 1;
                        String targetStr = line.substring(start);
                        for (MotorInfo motorInfo : motors) {
                            if (motorInfo != null && name.equals(motorInfo.name)) {
                                motorInfo.home = Integer.parseInt(homeStr);
                                motorInfo.target = Integer.parseInt(targetStr);
                                String str = String.format(Locale.ENGLISH, "%s,%d,%d\n", motorInfo.name, motorInfo.home, motorInfo.target);
                                Logger.message(str);
                                break;
                            }
                        }
                    }
                }

                line = reader.readLine();
            }

            reader.close();

        } catch(Exception e) {
            e.printStackTrace();
        }
    }
}

