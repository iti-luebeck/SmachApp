package de.uni_luebeck.iti.smachapp.model;

import android.graphics.Color;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class ColorActuator extends Actuator {

    private int defaultColor = Color.RED;

    public ColorActuator(String key, String topic, String topicType, String topicPackage, String objectInMessage) {
        super(key, topic, topicType, topicPackage, objectInMessage);
    }

    public ColorActuator(String key, String topic, String topicType, String topicPackage, String objectInMessage, int color) {
        super(key, topic, topicType, topicPackage, objectInMessage);
        defaultColor = color;
    }

    public void setDefaultColor(int color) {
        defaultColor = color;
    }

    public int getDefaultColor() {
        return defaultColor;
    }
}
