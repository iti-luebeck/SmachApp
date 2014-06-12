package de.uni_luebeck.iti.smachapp.utils;

import android.graphics.PointF;
import android.graphics.RectF;

/**
 * Created by Morten Mey on 24.05.2014.
 */
public class RectUtils {

    public static void extendRect(RectF rectangle, float margin) {
        rectangle.left -= margin;
        rectangle.right += margin;
        rectangle.top -= margin;
        rectangle.bottom += margin;
    }

    public static void makeRectFromPoint(PointF point, RectF result) {
        result.left = point.x;
        result.right = point.x;
        result.top = point.y;
        result.bottom = point.y;
    }
}
