package org.firstinspires.ftc.teamcode.opModes.autos.red.tests;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Writes the starting pose and alliance color to RobotSettings.json.
 * Run this at the end of autonomous to allow TeleOp to read the correct starting position.
 * This is the RED alliance version.
 */
@Autonomous(name = "Write Pose Red", group = "tests")
public class WritePoseRedAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Ready to write RED pose JSON");
        telemetry.update();

        waitForStart();

        JsonObject json = new JsonObject();
        json.addProperty("x",       124.62820249221184);
        json.addProperty("y",       117.35514018691589);
        json.addProperty("heading", Math.toRadians(143.0));
        json.addProperty("color",   "RED");

        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(json.toString());
            telemetry.addLine("RED pose written successfully");
            telemetry.addData("x",       124.62820249221184);
            telemetry.addData("y",       117.35514018691589);
            telemetry.addData("heading", "-37.0°");
            telemetry.addData("color",   "RED");
            telemetry.addData("file",    file.getAbsolutePath());
        } catch (IOException e) {
            telemetry.addLine("Failed to write JSON");
            telemetry.addData("error", e.getMessage());
        }

        telemetry.update();
        sleep(1000);
    }
}