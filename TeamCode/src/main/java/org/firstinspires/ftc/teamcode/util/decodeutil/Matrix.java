package org.firstinspires.ftc.teamcode.util.decodeutil;

public class Matrix {

    private final int rows;
    private final int cols;
    private final double[][] data;

    /* ---------- Constructors ---------- */

    // Create empty matrix (all zeros)
    public Matrix(int rows, int cols) {
        if (rows <= 0 || cols <= 0) {
            throw new IllegalArgumentException("Matrix dimensions must be positive");
        }
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];
    }

    // Create matrix from 2D array (deep copy)
    public Matrix(double[][] data) {
        if (data == null || data.length == 0 || data[0].length == 0) {
            throw new IllegalArgumentException("Invalid matrix data");
        }

        this.rows = data.length;
        this.cols = data[0].length;
        this.data = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            if (data[i].length != cols) {
                throw new IllegalArgumentException("Jagged arrays not allowed");
            }
            System.arraycopy(data[i], 0, this.data[i], 0, cols);
        }
    }

    /* ---------- Accessors ---------- */

    public int getRows() {
        return rows;
    }

    public int getCols() {
        return cols;
    }

    public double get(int r, int c) {
        return data[r][c];
    }

    public void set(int r, int c, double value) {
        data[r][c] = value;
    }

    /* ---------- Matrix Operations ---------- */

    // A + B
    public Matrix add(Matrix other) {
        checkSameSize(other);
        Matrix result = new Matrix(rows, cols);

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = this.data[i][j] + other.data[i][j];
            }
        }
        return result;
    }

    // A - B
    public Matrix subtract(Matrix other) {
        checkSameSize(other);
        Matrix result = new Matrix(rows, cols);

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = this.data[i][j] - other.data[i][j];
            }
        }
        return result;
    }

    // A × B
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException(
                    "Matrix multiplication dimension mismatch"
            );
        }

        Matrix result = new Matrix(this.rows, other.cols);

        for (int i = 0; i < this.rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                double sum = 0.0;
                for (int k = 0; k < this.cols; k++) {
                    sum += this.data[i][k] * other.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        return result;
    }

    // Aᵀ
    public Matrix transpose() {
        Matrix result = new Matrix(cols, rows);

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[j][i] = this.data[i][j];
            }
        }
        return result;
    }

    public Matrix inverse() {
        if (rows != cols) {
            throw new IllegalArgumentException("Only square matrices can be inverted");
        }

        int n = rows;
        double[][] augmented = new double[n][2 * n];

        // Build augmented matrix [A | I]
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented[i][j] = data[i][j];
            }
            augmented[i][i + n] = 1.0;
        }

        // Gauss-Jordan elimination
        for (int col = 0; col < n; col++) {

            // Find pivot
            int pivotRow = col;
            for (int row = col; row < n; row++) {
                if (Math.abs(augmented[row][col]) > Math.abs(augmented[pivotRow][col])) {
                    pivotRow = row;
                }
            }

            if (Math.abs(augmented[pivotRow][col]) < 1e-10) {
                throw new ArithmeticException("Matrix is singular and cannot be inverted");
            }

            // Swap rows
            double[] temp = augmented[col];
            augmented[col] = augmented[pivotRow];
            augmented[pivotRow] = temp;

            // Normalize pivot row
            double pivot = augmented[col][col];
            for (int j = 0; j < 2 * n; j++) {
                augmented[col][j] /= pivot;
            }

            // Eliminate other rows
            for (int row = 0; row < n; row++) {
                if (row != col) {
                    double factor = augmented[row][col];
                    for (int j = 0; j < 2 * n; j++) {
                        augmented[row][j] -= factor * augmented[col][j];
                    }
                }
            }
        }

        // Extract inverse matrix
        double[][] inverseData = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(augmented[i], n, inverseData[i], 0, n);
        }

        return new Matrix(inverseData);
    }

    public Matrix normalize() {
        double sumSq = 0.0;

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                sumSq += data[i][j] * data[i][j];
            }
        }

        double norm = Math.sqrt(sumSq);

        if (norm < 1e-12) {
            throw new ArithmeticException("Cannot normalize a zero matrix");
        }

        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result.data[i][j] = data[i][j] / norm;
            }
        }

        return result;
    }


    /* ---------- Helpers ---------- */

    private void checkSameSize(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions must match");
        }
    }

    /* ---------- Utility ---------- */

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < rows; i++) {
            sb.append("[ ");
            for (int j = 0; j < cols; j++) {
                sb.append(String.format("%8.3f", data[i][j])).append(" ");
            }
            sb.append("]\n");
        }
        return sb.toString();
    }
}
