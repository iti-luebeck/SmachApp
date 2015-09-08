/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
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

import de.uni_luebeck.iti.smachGenerator.ISmachableAction;
import de.uni_luebeck.iti.smachGenerator.ISmachableActuator;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class BeepColorRGBActuator implements ISmachableActuator {

    private int defaultColor = Color.RED;

    private String name;
    private String topic;
    private final int ledIndex;

    public BeepColorRGBActuator(String name, String topic, int ledIndex) {
        this.name = name;
        this.topic = topic;
        this.ledIndex = ledIndex;
    }

    public BeepColorRGBActuator(String name, String topic, int ledIndex, int defaultColor) {
        this(name, topic, ledIndex);
        this.defaultColor = defaultColor;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getPublisherSetup() {
        return "pub_led = rospy.Publisher('" + topic + "', Led)";
    }

    @Override
    public String getPublisherName() {
        return "pub_led";
    }

    @Override
    public String[] getPublishMessage(ISmachableAction a) {
        String[] res = new String[10];
        String col = "c" + ledIndex;
        String led = "led" + ledIndex;

        res[0] = col + " = MyColor()";
        res[1] = col + ".r = " + Color.red(a.getValue());
        res[2] = col + ".g = " + Color.green(a.getValue());
        res[3] = col + ".b = " + Color.blue(a.getValue());
        res[4] = led + " = Led()";
        res[5] = led + ".header.frame_id = 'led'";
        res[6] = led + ".header.stamp = rospy.get_rostime()";
        res[7] = led + ".col = " + col;
        res[8] = led + ".led = " + ledIndex;
        res[9] = getPublisherName() + ".publish(" + led + ")";

        return res;
    }

    @Override
    public Set<String> getImports() {
        HashSet<String> res = new HashSet<String>();
        res.add("from beep_msgs.msg import Led");
        res.add("from beep_msgs.msg import MyColor");
        return res;
    }

    @Override
    public String[] onShutDown() {
        Action stoppen = new Action("color", 0);
        return getPublishMessage(stoppen);
    }

    public void setDefaultColor(int color) {
        defaultColor = color;
    }

    public int getDefaultColor() {
        return defaultColor;
    }
}
