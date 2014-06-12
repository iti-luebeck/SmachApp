package de.uni_luebeck.iti.smachapp.utils;

import android.graphics.PointF;

/**
 * Created by Morten Mey on 24.05.2014.
 */
public class PointUtils {

    public static float distance(PointF a, PointF b) {
        return calculateDirection(a, b).length();
    }

    public static PointF calculateDirection(PointF from, PointF to) {
        return new PointF(to.x - from.x, to.y - from.y);
    }

    public static void normalize(PointF point) {
        float length = point.length();
        point.x /= length;
        point.y /= length;
    }
}
