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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.Locale;
import java.util.SortedSet;

import common.Logger;

/*
 * This OpMode calibrate any of the robot servos.
 */

@TeleOp(name="Calibrate Servo", group="Test")
@SuppressWarnings("unused")

public class CalibrateServo extends LinearOpMode {

    private final String calibrationPath = "/temp/servoCalibration.txt";

    private enum SERVO_SELECT { PREVIOUS, NEXT }

    private final ElapsedTime runtime = new ElapsedTime();

    private Servo servo = null;

    private static class ServoInfo {
        String  name;
        Servo   servo;
        double  home;
        double  target;
    }
    ServoInfo[] servos = new ServoInfo[12];
    int servoCount;
    int currentServo;

    @Override
    public void runOpMode() {

        getServos();
        readCalibration();

        telemetry.addLine("Press start");
        telemetry.update();
        telemetry.setAutoClear(false);

        waitForStart();

        Telemetry.Item servoNameMsg =  telemetry.addData("Servo name", 0);
        Telemetry.Item directionMsg = telemetry.addData("Servo direction", 0);
        Telemetry.Item positionMsg = telemetry.addData("Servo position", 0);
        Telemetry.Item homeMsg = telemetry.addData("Home position", 0);
        Telemetry.Item targetMsg = telemetry.addData("Target position", 0);

        setDisplayName(servoNameMsg);
        setDisplayDirection(directionMsg);

        telemetry.addData("\nServo Calibration Controls", "\n" +
                "  dpad left - select previous servo\n" +
                "  dpad right - select next servo\n" +
                "  left trigger - run servo backwards\n" +
                "  right trigger - run the servo forward\n" +
                "  left stick - increase/decrease target position\n" +
                "  right stick - increase/decrease home position\n" +
                "  y - set home position to current position\n" +
                "  a - set target position to current position\n" +
                "  x - run to home position\n" +
                "  b - run servo to target position\n" +
                "\n");

        telemetry.update();

        boolean save = true;

        while (opModeIsActive()) {

            if (gamepad1.a) {
                // set target position to current position
                servos[currentServo].target = servo.getPosition();

            } else if (gamepad1.y) {
                // set the home position to the current position
                servos[currentServo].home = servo.getPosition();

            } else if (gamepad1.x) {
                // run to zero position
                servo.setPosition(servos[currentServo].home);

            } else if (gamepad1.b) {
                // run servo to an target position
                servo.setPosition(servos[currentServo].target);

            } else if (gamepad1.left_trigger > 0) {
                // manually run the servo backwards
                while (gamepad1.left_trigger > 0) {
                    double position = servo.getPosition();
                    if (Double.isNaN(position)) {
                        Logger.message("Servo position not set");
                        break;
                    } else {
                        position -= 0.001;
                    }
                    servo.setPosition(position);
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }

            } else if (gamepad1.right_trigger > 0) {
                // manually run the motor forward
                while (gamepad1.right_trigger > 0) {
                    double position = servo.getPosition();
                    if (Double.isNaN(position)) {
                        Logger.message("Servo position not set");
                        break;
                    } else {
                        position += 0.001;
                    }
                    servo.setPosition(position);
                    setDisplayPosition(positionMsg);
                    telemetry.update();
                    sleep(100);
                }

            } else if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                runtime.reset();
                while (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ) {
                    if (gamepad1.left_stick_x > 0)
                        servos[currentServo].target = Math.min(1, servos[currentServo].target + incrementFine());
                    else if (gamepad1.left_stick_x < 0)
                        servos[currentServo].target = Math.max(0, servos[currentServo].target - incrementFine());
                    else if (gamepad1.left_stick_y < 0)
                        servos[currentServo].target = Math.min(1, servos[currentServo].target + incrementCoarse());
                    else if (gamepad1.left_stick_y > 0)
                        servos[currentServo].target = Math.max(0, servos[currentServo].target - incrementCoarse());
                    setDisplayTarget(targetMsg);
                    telemetry.update();
                }

            } else if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                runtime.reset();
                while (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0 ) {
                    if (gamepad1.right_stick_x > 0)
                        servos[currentServo].home = Math.min(1, servos[currentServo].home + incrementFine());
                    else if (gamepad1.right_stick_x < 0)
                        servos[currentServo].home = Math.max(0, servos[currentServo].home - incrementFine());
                    else if (gamepad1.right_stick_y < 0)
                        servos[currentServo].home = Math.min(1, servos[currentServo].home + incrementCoarse());
                    else if (gamepad1.right_stick_y > 0)
                        servos[currentServo].home = Math.max(0, servos[currentServo].home - incrementCoarse());
                    setDisplayHome(homeMsg);
                    telemetry.update();
                }

            } else if (gamepad1.dpad_left) {
                // select the next motor
                selectServo(SERVO_SELECT.PREVIOUS);
                while (gamepad1.dpad_left){
                    sleep(10);
                }
                setDisplayName(servoNameMsg);

            } else if (gamepad1.dpad_right) {
                // select the previous servo
                selectServo(SERVO_SELECT.NEXT);
                while (gamepad1.dpad_right) {
                    sleep(10);
                }
                setDisplayName(servoNameMsg);

            } else if (gamepad1.dpad_up) {
                // change the direction of the servo
                if (servo.getDirection() == Servo.Direction.FORWARD)
                    servo.setDirection(Servo.Direction.REVERSE);
                else
                    servo.setDirection(Servo.Direction.FORWARD);
                setDisplayDirection(directionMsg);
                while (gamepad1.dpad_up) sleep(10);

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

    private void setDisplayName (Telemetry.Item item) {
        item.setValue("%s  (port: %d)", servos[currentServo].name, servos[currentServo].servo.getPortNumber());
    }

    private void setDisplayDirection (Telemetry.Item item) {
        item.setValue("%s", servos[currentServo].servo.getDirection());
    }

    private void setDisplayPosition (Telemetry.Item item) {
        item.setValue( "%5.3f", servos[currentServo].servo.getPosition());
    }

    private void setDisplayHome (Telemetry.Item item) {
        item.setValue("%5.3f", servos[currentServo].home);
    }

    private void setDisplayTarget (Telemetry.Item item) {
        item.setValue("%5.3f", servos[currentServo].target);
    }


    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    private double incrementFine() {

        double incrementSlow = 0.001;
        double incrementMedium = 0.002;
        double incrementFast = 0.004;

        return increment (incrementSlow, incrementMedium, incrementFast);
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    private double incrementCoarse() {

        double incrementSlow = 0.01;
        double incrementMedium = 0.02;
        double incrementFast = 0.04;

        return increment (incrementSlow, incrementMedium, incrementFast);
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    private double increment(double incrementSlow, double incrementMedium, double incrementFast) {

        int sleepTime;
        double delta;

        if (runtime.seconds() < 3) {
            delta = incrementSlow;
            sleepTime = 500;
        } else if (runtime.seconds() < 6) {
            delta = incrementMedium;
            sleepTime = 500;
        } else if (runtime.seconds() < 9) {
            delta = incrementFast;
            sleepTime = 500;
        } else {
            delta = incrementFast;
            sleepTime = 250;
        }

        sleep(sleepTime);
        return delta;
    }

    /**
     * Build a list of servos sorted by port number
     */
    private void getServos(){

        servoCount = 0;
        SortedSet<String> names = hardwareMap.getAllNames(Servo.class);
        for (String name: names){
            Servo s = hardwareMap.get(Servo.class, name);
            servos[servoCount] = new ServoInfo();
            servos[servoCount].name = name;
            servos[servoCount].servo = s;
            servos[servoCount].home = 0.5;
            servos[servoCount].target = 0.5;
            servoCount++;
        }

        for (int i = 0; i < servos.length; i++) {
            if (servos[i] != null) {
                if (servo == null) {
                    servo = servos[i].servo;
                    currentServo = i;
                }
                Logger.message("servo name %s port %d", servos[i].name, servos[i].servo.getPortNumber());
            }
        }
    }

    private void selectServo (SERVO_SELECT select){
        int index;
        for (int i = 1; i <= servos.length; i++) {
            if (select == SERVO_SELECT.NEXT)
                index = (currentServo + i) %  servos.length;
            else
                index = (currentServo + servos.length - i) %  servos.length;

            if (servos[index] != null){
                servo = servos[index].servo;
                currentServo = index;
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

            for (ServoInfo servosInfo : servos) {
                if (servosInfo != null) {
                    String str = String.format(Locale.ENGLISH, "%s,%f,%f\n", servosInfo.name, servosInfo.home, servosInfo.target);
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
                Logger.message("Servo calibration file not found");
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
                        for (ServoInfo servosInfo : servos) {
                            if (servosInfo != null && name.equals(servosInfo.name)) {
                                servosInfo.home = Double.parseDouble(homeStr);
                                servosInfo.target = Double.parseDouble(targetStr);
                                String str = String.format(Locale.ENGLISH, "%s,%f,%f\n", servosInfo.name, servosInfo.home, servosInfo.target);
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

