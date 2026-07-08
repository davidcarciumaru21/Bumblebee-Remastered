package org.firstinspires.ftc.teamcode.global.configurations;

import com.pedropathing.geometry.Pose;

/**
 * Ballistic shooting constants.
 * Each inner class corresponds to a distance zone.
 * Modify SCORE_HEIGHT, SCORE_ANGLE and PASS_THROUGH_RADIUS per zone based on testing.
 * G is constant and shared across all zones.
 */
public class ShootingConfig {

    /** Gravitational acceleration in in/s². Constant, do not modify. */
    public static final double G = 386.1;

    /**
     * Close range shooting constants.
     * Used when distance < Close.MAX_DISTANCE.
     * SCORE_HEIGHT        — vertical distance from turret exit to goal in inches.
     * SCORE_ANGLE         — desired ball entry angle at goal in radians (relative to horizontal).
     * PASS_THROUGH_RADIUS — horizontal distance before goal where ball must have SCORE_ANGLE.
     * MAX_DISTANCE        — maximum distance in inches for this zone.
     */
    public static final class Close {
        public static final double SCORE_HEIGHT        = 40.0;
        public static final double SCORE_ANGLE         = Math.toRadians(-5);
        public static final double PASS_THROUGH_RADIUS = 5.0;
        public static final double MAX_DISTANCE        = 60.0;
    }

    /**
     * Mid range shooting constants.
     * Used when Close.MAX_DISTANCE <= distance < Mid.MAX_DISTANCE.
     * SCORE_HEIGHT        — vertical distance from turret exit to goal in inches.
     * SCORE_ANGLE         — desired ball entry angle at goal in radians (relative to horizontal).
     * PASS_THROUGH_RADIUS — horizontal distance before goal where ball must have SCORE_ANGLE.
     * MAX_DISTANCE        — maximum distance in inches for this zone.
     */
    public static final class Mid {
        public static final double SCORE_HEIGHT        = 40.0;
        public static final double SCORE_ANGLE         = Math.toRadians(-5);
        public static final double PASS_THROUGH_RADIUS = 5.0;
        public static final double MAX_DISTANCE        = 110.0;
    }

    /**
     * Far range shooting constants.
     * Used when distance >= Mid.MAX_DISTANCE.
     * SCORE_HEIGHT        — vertical distance from turret exit to goal in inches.
     * SCORE_ANGLE         — desired ball entry angle at goal in radians (relative to horizontal).
     * PASS_THROUGH_RADIUS — horizontal distance before goal where ball must have SCORE_ANGLE.
     */
    public static final class Far {
        public static final double SCORE_HEIGHT        = 48.0;
        public static final double SCORE_ANGLE         = Math.toRadians(-10);
        public static final double PASS_THROUGH_RADIUS = 0.0;
    }

    /**
     * Goal positions on the field for each alliance.
     * BLUE_GOAL_POSE      — Close and Mid target position on the blue alliance side.
     * RED_GOAL_POSE       — Close and Mid target position on the red alliance side.
     * BLUE_FAR_GOAL_POSE  — Far target position on the blue alliance side.
     * RED_FAR_GOAL_POSE   — Far target position on the red alliance side.
     * Coordinates are in inches, origin at bottom-left corner of the field.
     * Modify these values if the goal position changes between seasons.
     */
    public static final class Goals {

        public static final Pose RED_GOAL_POSE       = new Pose(137, 138);
        public static final Pose BLUE_GOAL_POSE      = new Pose(3, 138);
        public static final Pose RED_FAR_GOAL_POSE   = new Pose(137, 138);
        public static final Pose BLUE_FAR_GOAL_POSE  = new Pose(3, 138);
    }
}
