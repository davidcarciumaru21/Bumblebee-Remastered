package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

public final class AutoUtils {

    private AutoUtils() {}

    public static double mirrorHeading(double deg) {
        return ((180.0 - deg) % 360.0 + 360.0) % 360.0;
    }

    public static Command followWithTimeout(Follower follower, PathChain path, double timeoutMs) {
        return race(
                follow(follower, path),
                timeoutStop(follower, timeoutMs)
        );
    }

    public static Command followWithTimeout(
            Follower follower,
            PathChain path,
            boolean holdEnd,
            double maxPower,
            double timeoutMs
    ) {
        return race(
                follow(follower, path, holdEnd, maxPower),
                timeoutStop(follower, timeoutMs)
        );
    }

    private static Command timeoutStop(Follower follower, double timeoutMs) {
        return sequential(
                waitMs(timeoutMs),
                instant(follower::breakFollowing)
        );
    }
}
