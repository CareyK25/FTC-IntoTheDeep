package org.firstinspires.ftc.teamcode.rendertypes;

import org.firstinspires.ftc.teamcode.datatypes.Pair;

import java.util.ArrayList;
import java.util.Arrays;

//all actual units are in cm
public class GameMap {
    private static final Pair FIELD_DIMENSIONS = new Pair(0, 0);// DO IT IN CM
    private char[][] map;
    private Pair resolution;
    private ArrayList<Line> lines = new ArrayList<>();


    public GameMap(Pair resolution) {
        this.resolution = resolution;
        this.map = new char[(int)resolution.getX()][(int)resolution.getY()];


    }

    public void renderBuffer(Pair res) {
        for (Line line : lines) {
            drawPixels(line.getRasterized(res).render());// rasterize to new size then render the line
        }
    }

    public void drawPixels(ArrayList<Pixel> pixels) {
        for (Pixel pixel : pixels) {
            map[pixel.getY()][pixel.getX()] = pixel.getValue();
        }
    }


    public char[][] sampleImage(Pair offset, int height, int width) {
        char[][] snip = new char[height][width];
        for (int y = 0; y<height; y++) {
            for (int x = 0; x<width; x++) {
                snip[y][x] = map[(int)offset.getY() + y][(int)offset.getX()+x];
            }
        }
        return snip;
    }
}
