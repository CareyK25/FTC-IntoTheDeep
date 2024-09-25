package org.firstinspires.ftc.teamcode.rendertypes;

import org.firstinspires.ftc.teamcode.datatypes.Pair;

import java.util.ArrayList;

public class BoundingBox {
    private Pair[] vertices;

    private double rotation;
    public BoundingBox(Pair[] vertices) {
        this.vertices = vertices;
    }

    public void rotate(double radians) {
        rotation += radians-rotation;
        for (Pair vertex : vertices) {
            vertex.rotate(radians-rotation);
        }
    }


    public ArrayList<Pixel> render() {
        ArrayList<Pixel> pixels = new ArrayList<>();
        for (int i = 1; i<vertices.length; i++) {
            pixels.addAll(Line.render(vertices[i-1], vertices[i]));
        }
        return pixels;
    }


}
