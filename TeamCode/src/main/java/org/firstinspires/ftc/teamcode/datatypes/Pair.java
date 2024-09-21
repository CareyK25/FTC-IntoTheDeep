package org.firstinspires.ftc.teamcode.datatypes;

public class Pair {
    public double x, y;

    public Pair(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void add(Pair p2) {
        this.x += p2.getX();
        this.y += p2.getY();
    }
}
