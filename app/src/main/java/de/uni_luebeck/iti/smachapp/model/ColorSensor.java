package de.uni_luebeck.iti.smachapp.model;

import android.graphics.Color;

import de.uni_luebeck.iti.smachGenerator.Operator;

/**
 * Created by Morten Mey on 07.06.2014.
 */
public class ColorSensor extends Sensor {

    private int defaultColor=Color.RED;

    public ColorSensor(String key, String topic, String topicType, String topicPackage, String objectInMessage) {
        super(key, topic, topicType, topicPackage, objectInMessage);
    }

    public ColorSensor(String key, String topic, String topicType, String topicPackage, String objectInMessage, int defaultColor) {
        super(key, topic, topicType, topicPackage, objectInMessage);
        this.defaultColor=defaultColor;
    }

    public int getDefaultColor() {
        return defaultColor;
    }

    public void setDefaultColor(int color) {
        defaultColor = color;
    }

    @Override
    public String getTransitionCondition(Operator op, int compVal) {
        float[] hsbCol = RGBtoHSB(Color.red(compVal), Color.green(compVal), Color.blue(compVal), null);

        return key + ">" + (hsbCol[0] - 0.1 + 1) % 1 + " and " + key + "<" + (hsbCol[0] + 0.1) % 1;
    }

    public static float[] RGBtoHSB(int red, int green, int blue, float[] array) {
        if (array == null)
            array = new float[3];
        // Calculate brightness.
        int min;
        int max;
        if (red < green) {
            min = red;
            max = green;
        } else {
            min = green;
            max = red;
        }
        if (blue > max)
            max = blue;
        else if (blue < min)
            min = blue;
        array[2] = max / 255f;
        // Calculate saturation.
        if (max == 0)
            array[1] = 0;
        else
            array[1] = ((float) (max - min)) / ((float) max);
        // Calculate hue.
        if (array[1] == 0)
            array[0] = 0;
        else {
            float delta = (max - min) * 6;
            if (red == max)
                array[0] = (green - blue) / delta;
            else if (green == max)
                array[0] = 1f / 3 + (blue - red) / delta;
            else
                array[0] = 2f / 3 + (red - green) / delta;
            if (array[0] < 0)
                array[0]++;
        }
        return array;
    }
}
