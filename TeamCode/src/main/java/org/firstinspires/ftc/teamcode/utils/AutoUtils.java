package org.firstinspires.ftc.teamcode.utils;

public final class AutoUtils {

    private AutoUtils() {}

    public static double mirrorHeading(double deg) {
        return ((180.0 - deg) % 360.0 + 360.0) % 360.0;
    }
}
