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

import org.apache.http.conn.util.InetAddressUtils;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URI;
import java.util.Enumeration;

import beep_msgs.Color_sensors;
import beep_msgs.IR;
import de.uni_luebeck.iti.smachGenerator.ISmachableSensor;
import smach_msgs.SmachContainerStatus;

/**
 * Created by Morten Mey on 12.07.2014.
 */
public class RosNode extends AbstractNodeMain {

    private DebugModel model;
    private NodeMainExecutor executer;
    private String ip;

    public RosNode(DebugModel model, String ip) {
        this.model = model;
        this.ip = "http://"+ip+":11311/";
        executer = DefaultNodeMainExecutor.newDefault();
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        for (ISmachableSensor sen : model.getEditor().getRobot().getIntSensors()) {
            Subscriber<IR> sub = connectedNode.newSubscriber(sen.getTopic(), sen.getTopicType());

            sub.addMessageListener(new MessageListener<IR>() {
                @Override
                public void onNewMessage(IR m) {
                    model.updateIR(m.getIr());
                }
            });
        }

        for (ISmachableSensor sen : model.getEditor().getRobot().getColorSensors()) {
            Subscriber<Color_sensors> sub = connectedNode.newSubscriber(sen.getTopic(), sen.getTopicType());

            sub.addMessageListener(new MessageListener<Color_sensors>() {
                @Override
                public void onNewMessage(Color_sensors color_sensors) {
                    model.updateGroundColors(color_sensors.getSensors());
                }
            });
        }

        Subscriber<SmachContainerStatus> sub = connectedNode.newSubscriber("/Beep_State_Server/smach/container_status", SmachContainerStatus._TYPE);

        sub.addMessageListener(new MessageListener<SmachContainerStatus>() {
            @Override
            public void onNewMessage(SmachContainerStatus smachContainerStatus) {
                if (smachContainerStatus.getActiveStates().size() == 1) {
                    model.updateCurrentState(smachContainerStatus.getActiveStates().get(0));
                } else {
                    System.out.println("More then one State Active!");
                }
            }
        });
    }

    public static String getLocalIpAddress()
    {
        try
        {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();)
            {
                NetworkInterface intf = en.nextElement();
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();)
                {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress() && InetAddressUtils.isIPv4Address(inetAddress.getHostAddress()))
                    {
                        return inetAddress.getHostAddress();
                    }
                }
            }
        }
        catch (SocketException ex)
        {
            ex.printStackTrace();
        }
        return "";
    }

    public void startNode() {
        URI muri = URI.create(ip);
        String myIp=getLocalIpAddress();
        NodeConfiguration config = NodeConfiguration.newPublic(myIp, muri); //Node runs on myIp, Master is at muri
        config.setNodeName("SmachAppNode");
        executer.execute(this, config);
    }

    public void stopNode() {
        executer.shutdown();
    }
}
