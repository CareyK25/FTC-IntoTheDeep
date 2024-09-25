package org.firstinspires.ftc.teamcode.rendertypes;

import org.firstinspires.ftc.teamcode.datatypes.Pair;

import java.util.ArrayList;

public class Line {
    private Pair start;
    private Pair end;

    public static final char[] textures = {'\\', '-', '|', '/'};
    // in order   [ \, -, |, /]  (slope left, horizontal, vertical, slope right)

    public Line(Pair start, Pair end) {
        this.start = start;
        this.end = end;
    }

    public ArrayList<Pixel> render() {
        return Line.render(this.start, this.end);
    }

    public Pair getStart() {
        return start;
    }

    public void setStart(Pair start) {
        this.start = start;
    }

    public Pair getEnd() {
        return end;
    }

    public void setEnd(Pair end) {
        this.end = end;
    }
    public void normalize(double height, double width) {
        start.normalize(height, width);
        end.normalize(height, width);
    }

    public Line getRasterized(double height, double width) {
        return new Line(start.getRasterized(height, width), end.getRasterized(height, width));
    }
    public Line getRasterized(Pair dim) {
        return getRasterized(dim.getY(), dim.getX());
    }

    //implementation of brezenham's line algorithm
    public static ArrayList<Pixel> render(Pair start, Pair end) { // modified Brezenham line algorithm
        ArrayList<Pixel> lineCoords = new ArrayList<>();// pixels that make up the line

        //lintex array is a preset format from blocks that defines the line textures at certain slopes


        int x1 = (int)start.getX();// start x
        int y1 = (int)start.getY();// start y

        int x2 = (int)end.getX();// end x
        int y2 = (int)end.getY();// end y


        // delta of exact value and rounded value of the dependent variable
        int d = 0;

        int dx = Math.abs(x2 - x1);
        int dy = Math.abs(y2 - y1);

        int dx2 = 2 * dx; // slope scaling factors to
        int dy2 = 2 * dy; // avoid floating point

        int ix = x1 < x2 ? 1 : -1; // increment direction
        int iy = y1 < y2 ? 1 : -1;

        int x = x1;
        int y = y1;

        char lineChar = 'E';// init to something, could be anything
        boolean isSlope = false;

        if (dx > dy) {
            while (true) {
                lineCoords.add(new Pixel(x, y, lineChar));
                if (x == x2)
                    break;
                x += ix;
                d += dy2;
                if (d > dx) {
                    y += iy;
                    d -= dx2;
                }
            }
        } else {
            while (true) {
                if (!isSlope) {
                    lineChar = textures[2];
                }
                lineCoords.add(new Pixel(x, y, lineChar));
                if (y == y2)
                    break;

                y += iy;
                d += dx2;

                if (d > dy) {
                    lineChar = ix != 1 ? textures[3] : textures[0];
                    isSlope = true;
                    x += ix;
                    d -= dy2;
                }
            }
        }

        return lineCoords;
    }


}
