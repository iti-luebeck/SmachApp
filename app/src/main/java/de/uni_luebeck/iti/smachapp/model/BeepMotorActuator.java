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

import java.util.HashSet;

import de.uni_luebeck.iti.smachGenerator.ISmachableAction;
import de.uni_luebeck.iti.smachGenerator.ISmachableActuator;

/**
 * Created by Morten Mey on 18.05.2014.
 */
public class BeepMotorActuator implements ISmachableActuator {

    private int min = -128;
    private int max = 127;

    private String name;
    private String topic;
    private final String topicType = "Int8";

    public BeepMotorActuator(String name, String topic) {
        this.name = name;
        this.topic = topic;
    }


    @Override
    public String getName() {
        return name;
    }

    @Override
    public String getPublisherSetup() {
        return getPublisherName() + " = rospy.Publisher('" + topic
                + "', " + topicType + ")";
    }

    @Override
    public String[] getPublishMessage(ISmachableAction a) {
        String[] result = new String[3];
        result[0] = name + " = " + topicType + "()";
        result[1] = name + ".data = " + a.getValue();
        result[2] = getPublisherName() + ".publish(" + name + ")";
        return result;
    }

    @Override
    public HashSet<String> getImports() {
        HashSet<String> result = new HashSet<String>();
        result.add("from std_msgs.msg import " + topicType);
        return result;
    }

    @Override
    public String getPublisherName() {
        return "pub_" + name;
    }

    @Override
    public String[] onShutDown() {
        String[] commands = new String[1];
        commands[0] = getPublisherName() + ".publish(0)";
        return commands;
    }

    public void setMin(int min) {
        this.min = min;
    }

    public void setMax(int max) {
        this.max = max;
    }

    public int getMin() {
        return min;
    }

    public int getMax() {
        return max;
    }
}
