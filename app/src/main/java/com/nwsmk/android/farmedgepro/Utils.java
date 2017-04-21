package com.nwsmk.android.farmedgepro;

/**
 * Created by nwsmk on 3/24/2017.
 */

public class Utils {
    /** Finding mean of an array */
    public static double mean(double[] m) {
        double sum = 0;
        for (int i = 0; i < m.length; i++) {
            sum += m[i];
        }
        return sum / m.length;
    }

    /** Sum of a matrix */
    public static int sum2D(int[][] m) {
        int sum = 0;

        int num_rows = m.length;
        int num_cols = m[0].length;

        for (int i = 0; i < num_rows; i++) {
            for (int j = 0; j < num_cols; j++) {
                sum = sum + m[i][j];
            }
        }
        return sum;
    }
}
