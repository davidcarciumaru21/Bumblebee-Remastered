package org.firstinspires.ftc.teamcode.opModes.autos.tests.blue;

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
 * This is the BLUE alliance version.
 */
@Autonomous(name = "Write Pose Blue", group = "tests")
public class WritePoseBlueAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Ready to write BLUE pose JSON");
        telemetry.update();

        waitForStart();

        JsonObject json = new JsonObject();
        json.addProperty("x",       31.176489096573214);
        json.addProperty("y",       132.12227414330215);
        json.addProperty("heading", Math.toRadians(270));
        json.addProperty("color",   "BLUE");
        json.addProperty("turret", 0.0);

        File file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            writer.write(json.toString());
            telemetry.addLine("BLUE pose written successfully");
            telemetry.addData("x",       31.176489096573214);
            telemetry.addData("y",       132.5630841121495);
            telemetry.addData("heading", "-270°");
            telemetry.addData("color",   "BLUE");
            telemetry.addData("file",    file.getAbsolutePath());
        } catch (IOException e) {
            telemetry.addLine("Failed to write JSON");
            telemetry.addData("error", e.getMessage());
        }

        telemetry.update();
        sleep(1000);
    }
}