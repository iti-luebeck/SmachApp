/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of L�beck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package de.uni_luebeck.iti.smachapp.model;

import android.graphics.Color;

import java.util.HashSet;
import java.util.Set;

import beep_msgs.Color_sensors;
import de.uni_luebeck.iti.smachGenerator.ISmachableSensor;

/**
 * Created by Morten Mey on 07.06.2014.
 */
public class BeepColorSensor implements ISmachableSensor {

    private int defaultColor = Color.RED;
    private String name;
    private String topic;
    private final String topicType = "beep_msgs/Color_sensors";
    private final int sensorIndex;

    public BeepColorSensor(String name, String topic, int index) {
        this.name = name;
        this.topic = topic;
        this.sensorIndex = index;
    }

    public BeepColorSensor(String name, String topic, int index, int defaultColor) {
        this(name, topic, index);
        this.defaultColor = defaultColor;
    }

    public int getDefaultColor() {
        return defaultColor;
    }

    public void setDefaultColor(int color) {
        defaultColor = color;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Set<String> getImports() {
        HashSet<String> res = new HashSet<String>();
        res.add("from " + topicType.split("/")[0] + ".msg import "
                + topicType.split("/")[1]);
        res.add("import colorsys");
        return res;
    }

    @Override
    public String getCallback() {
        String res = "";
        res += "def color_cb(msg):\n";
        res += "\tglobal colorSensor\n";
        res += "\tfor (i, sensor) in enumerate(msg.sensors):\n";
        res += "\t\tcolorSensor[i] = colorsys.rgb_to_hsv(sensor.r, sensor.g, sensor.b)[0]\n";
        return res;
    }

    @Override
    public String getSubscriberSetup() {
        String res = "";
        res += "rospy.Subscriber('" + topic + "', " + topicType.split("/")[1]
                + ", color_cb)\n";
        return res;
    }

    @Override
    public String getValueIdentifier() {
        return "colorSensor[" + sensorIndex + "]";
    }

    @Override
    public String getGlobalIdentifier() {
        return "colorSensor";
    }

    @Override
    public String getIdentifierInit() {
        return "colorSensor = [0, 0, 0]";
    }

    @Override
    public String getTransitionCondition(String op, int compVal) {
        float[] hsbCol = RGBtoHSB(Color.red(compVal), Color.green(compVal), Color.blue(compVal));

        return getValueIdentifier() + ">" + (hsbCol[0] - 0.1 + 1) % 1 + " and "
                + getValueIdentifier() + "<" + (hsbCol[0] + 0.1) % 1;
    }

    @Override
    public String[] onShutDown() {
        return new String[0];
    }

    @Override
    public String getTopic() {
        return "/ground_colors";
    }

    @Override
    public String getTopicType() {
        return Color_sensors._TYPE;
    }

    private static float[] RGBtoHSB(int red, int green, int blue) {

        float[] array = new float[3];
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
