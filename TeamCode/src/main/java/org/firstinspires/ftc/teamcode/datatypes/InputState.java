package org.firstinspires.ftc.teamcode.datatypes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputState {
    long timestamp;

    boolean a, b, x, y;
    boolean cross, circle, square, triangle;

    boolean left_bumper, right_bumper;

    float left_trigger, right_trigger;

    boolean dpad_up, dpad_down, dpad_left, dpad_right;

    boolean guide, share;

    Pair left_stick, right_stick;

    boolean left_touchpad, right_touchpad;
    Pair left_touchpad_pos, right_touchpad_pos;

    long nextRumbleApproxFinishTime;

    public InputState(Gamepad gp) {
        timestamp = gp.timestamp;

        a = gp.a;
        cross = a;

        b = gp.b;
        circle = b;

        x = gp.x;
        square = x;

        y= gp.y;
        triangle = y;

        left_bumper = gp.left_bumper;
        right_bumper = gp.right_bumper;

        left_trigger = gp.left_trigger;
        right_trigger = gp.right_trigger;

        dpad_up = gp.dpad_up;
        dpad_down = gp.dpad_down;
        dpad_left = gp.dpad_left;
        dpad_right = gp.dpad_right;

        guide = gp.guide;
        share = gp.share;

        left_stick = new Pair(gp.left_stick_x, gp.left_stick_y);
        right_stick = new Pair(gp.left_stick_x, gp.left_stick_y);

    }
}
