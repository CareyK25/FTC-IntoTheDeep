package org.firstinspires.ftc.teamcode.datatypes;

import java.lang.reflect.Array;
import java.util.Arrays;

public class Matrix {
    private double[][] matrix;



    public Matrix(double[][] matrix) {
        this.matrix = matrix;
    }

    public int getHeight() {
        return matrix.length;
    }
    public int getWidth() {
        return matrix[0].length;
    }
    public double[][] getMatrix() {
        return matrix;
    }

    public void setMatrix(double[][] matrix) {
        this.matrix = matrix;
    }

    public Matrix multiply(Matrix m2) {
        if (!(this.getWidth() == m2.getHeight())) {
            return null; // cannot multiply these matrices
        }

        double[][] m1 = this.matrix; // rename it locally so its easier to read the loop
        double[][] product = new double[this.getHeight()][m2.getWidth()];

        double accumulation =0;
        for (int r = 0; r<this.getHeight(); r++) {
            for (int x = 0; x<m2.getWidth(); x++) {
                accumulation = 0;
                for (int y = 0; y<m2.getHeight(); y++) {
                    accumulation += this.matrix[r][y]*m2.getMatrix()[y][x];
                }
                product[r][x] = accumulation;
            }
        }
        return new Matrix(product);
    }
}


