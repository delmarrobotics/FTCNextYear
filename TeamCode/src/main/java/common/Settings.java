package common;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;

public class Settings {
    private static final String settingsPath = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "/temp/Config.txt");

    public static class MotorSettings {
        String      name;
        int         home;
        int         target;

        private MotorSettings (String name, int home, int target) {
            this.name = name;
            this.home = home;
            this.target = target;
        }
    }

    public static class DriveSettings {
        public double speedMin;
        public double speedMax;
        public double driveFactor;
        public double strafeFactor;
        public double turnFactor;
    }

    DriveSettings driveSettings;
    MotorSettings motorSettings[];
    ArrayList<MotorSettings> m;

    public static Settings settings;

    private static void initSettings () {
        settings = new Settings();

        settings.driveSettings = new DriveSettings();
        settings.driveSettings.speedMin = 0.2;
        settings.driveSettings.speedMax = 0.95;
        settings.driveSettings.driveFactor = 1;
        settings.driveSettings.strafeFactor = 1;
        settings.driveSettings.turnFactor = 10;

        settings.motorSettings = new MotorSettings[3];
        settings.motorSettings[0] = new MotorSettings("motorOne", 100, 200);
        settings.motorSettings[1] = new MotorSettings("motorTwo", 100, 200);

        settings.m = new ArrayList<>();
        settings.m.add(new MotorSettings("motorThree", 100, 200));
        //settings.m.add(new MotorSettings("motorFour", 100, 200));
    }

    private static void readSettings() {

        if (settings == null) {
            try {
                BufferedReader br = new BufferedReader(new FileReader(settingsPath));
                settings = new Gson().fromJson(br, Settings.class);
            } catch (Exception e) {
                Logger.error(e, "Error reading settings");
                initSettings();
            }
        }
    }

    public static void writeSettings ()  {

        try {
            File file = new File(settingsPath);
            Logger.message("%s exist is %b", settingsPath, file.exists());
            if (file.exists()) {
                String path = String.format("%s%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "/temp/Config.bak");
                boolean renamed = file.renameTo(new File(path));
                if (renamed)
                    Logger.message("%s renamed to %s", settingsPath, path);
            }

            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            FileWriter f = new FileWriter(settingsPath, false);

            gson.toJson(settings, f);
            f.flush();
            f.close();

        } catch (Exception e) {
            Logger.error(e, "Error writing settings");
        }
    }

    public static double getDriveFactor() {
        readSettings();
        return settings.driveSettings.driveFactor;
    }

    public static void setDriveFactor(double driveFactor) {
        readSettings();
        settings.driveSettings.driveFactor = driveFactor;
    }

    public static double getStrafeFactor() {
        readSettings();
        return settings.driveSettings.strafeFactor;
    }

    public static void setStrafeFactor(double strafeFactor) {
        readSettings();
        settings.driveSettings.strafeFactor = strafeFactor;
    }

    public static double getTurnFactor() {
        readSettings();
        return settings.driveSettings.turnFactor;
    }

    public static void setTurnFactor(double turnFactor) {
        readSettings();
        settings.driveSettings.turnFactor = turnFactor;
    }
}
