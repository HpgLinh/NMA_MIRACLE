package org.firstinspires.ftc.teamcode.phoenix;
import org.firstinspires.ftc.teamcode.phoenix.geometry.Pose;

public class constants_numbers{
    public static final Pose blueup_start = new Pose(32,135,Math.toRadians(180));
    public static final Pose bluedown_start = new Pose(50, 9, Math.toRadians(180));
    
    public static final Pose redup_start = new Pose(115, 135, 0); //radians
    public static final Pose reddown_start = new Pose(90, 9, 0);
    
    public static double BLUE_GOAL_X = 10, BLUE_GOAL_Y = 135;
    public static double RED_GOAL_X = 135, RED_GOAL_Y = 138;
    
    public static double RED_PARKING_X = 38.624, RED_PARKING_Y = 33.265;
    public static double BLUE_PARKING_X = 105.154, BLUE_PARKING_Y = 33.265;
}