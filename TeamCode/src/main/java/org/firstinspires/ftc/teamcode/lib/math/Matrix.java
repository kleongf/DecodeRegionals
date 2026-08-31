package org.firstinspires.ftc.teamcode.lib.math;

import java.util.Arrays;

public class Matrix {

    private final double[][] data;
    private final int rows;
    private final int cols;

    // -------------------------------------------------------------------------
    // Constructors
    // -------------------------------------------------------------------------

    /**
     * Creates a Matrix from a 2-D array.
     * The array is defensively copied, so later mutations of the source have no effect.
     *
     * @param data a non-null, non-empty, rectangular 2-D array
     * @throws IllegalArgumentException if the array is null, empty, or jagged
     */
    public Matrix(double[][] data) {
        if (data == null || data.length == 0 || data[0].length == 0) {
            throw new IllegalArgumentException("Matrix data must be non-null and non-empty.");
        }
        int numCols = data[0].length;
        for (double[] row : data) {
            if (row.length != numCols) {
                throw new IllegalArgumentException("All rows must have the same number of columns (jagged array detected).");
            }
        }
        this.rows = data.length;
        this.cols = numCols;
        // Defensive copy
        this.data = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            this.data[i] = Arrays.copyOf(data[i], cols);
        }
    }

    /**
     * Creates a zero matrix of the given dimensions.
     */
    public Matrix(int rows, int cols) {
        if (rows <= 0 || cols <= 0) {
            throw new IllegalArgumentException("Dimensions must be positive.");
        }
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];
    }

    // -------------------------------------------------------------------------
    // Factory helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the n×n identity matrix.
     */
    public static Matrix identity(int n) {
        Matrix m = new Matrix(n, n);
        for (int i = 0; i < n; i++) m.data[i][i] = 1.0;
        return m;
    }

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    public int getRows() {
        return rows;
    }

    public int getCols() {
        return cols;
    }

    public double get(int row, int col) {
        checkBounds(row, col);
        return data[row][col];
    }

    /**
     * Returns a deep copy of the underlying data array.
     */
    public double[][] toArray() {
        double[][] copy = new double[rows][cols];
        for (int i = 0; i < rows; i++) copy[i] = Arrays.copyOf(data[i], cols);
        return copy;
    }

    // -------------------------------------------------------------------------
    // Addition
    // -------------------------------------------------------------------------

    /**
     * Returns {@code this + other}.
     *
     * @throws IllegalArgumentException if dimensions don't match
     */
    public Matrix add(Matrix other) {
        checkSameDimensions(other);
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result.data[i][j] = this.data[i][j] + other.data[i][j];
        return result;
    }

    // -------------------------------------------------------------------------
    // Subtraction
    // -------------------------------------------------------------------------

    /**
     * Returns {@code this - other}.
     *
     * @throws IllegalArgumentException if dimensions don't match
     */
    public Matrix subtract(Matrix other) {
        checkSameDimensions(other);
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result.data[i][j] = this.data[i][j] - other.data[i][j];
        return result;
    }

    // -------------------------------------------------------------------------
    // Scalar multiplication
    // -------------------------------------------------------------------------

    /**
     * Returns {@code scalar * this}.
     */
    public Matrix scalarMultiply(double scalar) {
        Matrix result = new Matrix(rows, cols);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result.data[i][j] = scalar * this.data[i][j];
        return result;
    }

    // -------------------------------------------------------------------------
    // Matrix multiplication
    // -------------------------------------------------------------------------

    /**
     * Returns {@code this × other} (standard matrix product).
     *
     * @throws IllegalArgumentException if this.cols != other.rows
     */
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException(
                    String.format("Cannot multiply: (%dx%d) × (%dx%d) — inner dimensions must match.",
                            this.rows, this.cols, other.rows, other.cols));
        }
        Matrix result = new Matrix(this.rows, other.cols);
        for (int i = 0; i < this.rows; i++)
            for (int j = 0; j < other.cols; j++)
                for (int k = 0; k < this.cols; k++)
                    result.data[i][j] += this.data[i][k] * other.data[k][j];
        return result;
    }

    // -------------------------------------------------------------------------
    // Transposition
    // -------------------------------------------------------------------------

    /**
     * Returns the transpose of this matrix.
     */
    public Matrix transpose() {
        Matrix result = new Matrix(cols, rows);
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                result.data[j][i] = this.data[i][j];
        return result;
    }

    // -------------------------------------------------------------------------
    // Determinant
    // -------------------------------------------------------------------------

    /**
     * Returns the determinant of this matrix via LU decomposition.
     *
     * @throws IllegalStateException if the matrix is not square
     */
    public double determinant() {
        requireSquare();
        // LU decomposition with partial pivoting
        double[][] lu = deepCopy(data);
        int n = rows;
        int[] pivot = new int[n];
        int sign = luDecompose(lu, pivot, n);
        if (sign == 0) return 0.0; // singular

        double det = sign;
        for (int i = 0; i < n; i++) det *= lu[i][i];
        return det;
    }

    // -------------------------------------------------------------------------
    // Inversion
    // -------------------------------------------------------------------------

    /**
     * Returns the inverse of this matrix via Gauss–Jordan elimination with
     * partial pivoting.
     *
     * @throws IllegalStateException if the matrix is not square
     * @throws ArithmeticException   if the matrix is singular
     */
    public Matrix inverse() {
        requireSquare();
        int n = rows;

        // Augmented matrix [this | I]
        double[][] aug = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) aug[i][j] = data[i][j];
            aug[i][n + i] = 1.0;
        }

        // Forward elimination with partial pivoting
        for (int col = 0; col < n; col++) {
            // Find pivot
            int pivotRow = -1;
            double maxVal = 1e-12; // treat smaller values as zero
            for (int row = col; row < n; row++) {
                if (Math.abs(aug[row][col]) > maxVal) {
                    maxVal = Math.abs(aug[row][col]);
                    pivotRow = row;
                }
            }
            if (pivotRow == -1) {
                throw new ArithmeticException("Matrix is singular and cannot be inverted.");
            }
            // Swap rows
            double[] tmp = aug[col];
            aug[col] = aug[pivotRow];
            aug[pivotRow] = tmp;

            // Scale pivot row
            double pivotVal = aug[col][col];
            for (int j = 0; j < 2 * n; j++) aug[col][j] /= pivotVal;

            // Eliminate column entries in all other rows
            for (int row = 0; row < n; row++) {
                if (row == col) continue;
                double factor = aug[row][col];
                for (int j = 0; j < 2 * n; j++) aug[row][j] -= factor * aug[col][j];
            }
        }

        // Extract right half
        Matrix inv = new Matrix(n, n);
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++)
                inv.data[i][j] = aug[i][n + j];
        return inv;
    }

    // -------------------------------------------------------------------------
    // toString / equals / hashCode
    // -------------------------------------------------------------------------

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < rows; i++) {
            sb.append("[ ");
            for (int j = 0; j < cols; j++) {
                sb.append(String.format("%10.4f", data[i][j]));
                if (j < cols - 1) sb.append(", ");
            }
            sb.append(" ]");
            if (i < rows - 1) sb.append("\n");
        }
        return sb.toString();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Matrix)) return false;
        Matrix other = (Matrix) obj;
        if (rows != other.rows || cols != other.cols) return false;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                if (Math.abs(data[i][j] - other.data[i][j]) > 1e-10) return false;
        return true;
    }

    @Override
    public int hashCode() {
        return Arrays.deepHashCode(data);
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    private void checkBounds(int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols)
            throw new IndexOutOfBoundsException(
                    String.format("Index (%d, %d) out of bounds for %dx%d matrix.", row, col, rows, cols));
    }

    private void checkSameDimensions(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols)
            throw new IllegalArgumentException(
                    String.format("Dimension mismatch: (%dx%d) vs (%dx%d).", rows, cols, other.rows, other.cols));
    }

    private void requireSquare() {
        if (rows != cols)
            throw new IllegalStateException(
                    String.format("Operation requires a square matrix, but got %dx%d.", rows, cols));
    }

    private static double[][] deepCopy(double[][] src) {
        double[][] copy = new double[src.length][];
        for (int i = 0; i < src.length; i++) copy[i] = Arrays.copyOf(src[i], src[i].length);
        return copy;
    }

    /**
     * In-place LU decomposition with partial pivoting (Doolittle's algorithm).
     * Returns the sign of the permutation (+1 or -1), or 0 if singular.
     */
    private static int luDecompose(double[][] a, int[] pivot, int n) {
        int sign = 1;
        for (int i = 0; i < n; i++) pivot[i] = i;

        for (int col = 0; col < n; col++) {
            // Partial pivoting
            double maxVal = 0;
            int maxRow = col;
            for (int row = col; row < n; row++) {
                if (Math.abs(a[row][col]) > maxVal) {
                    maxVal = Math.abs(a[row][col]);
                    maxRow = row;
                }
            }
            if (maxVal < 1e-12) return 0; // singular

            if (maxRow != col) {
                double[] tmp = a[col];
                a[col] = a[maxRow];
                a[maxRow] = tmp;
                int t = pivot[col];
                pivot[col] = pivot[maxRow];
                pivot[maxRow] = t;
                sign = -sign;
            }

            for (int row = col + 1; row < n; row++) {
                a[row][col] /= a[col][col];
                for (int k = col + 1; k < n; k++)
                    a[row][k] -= a[row][col] * a[col][k];
            }
        }
        return sign;
    }
}