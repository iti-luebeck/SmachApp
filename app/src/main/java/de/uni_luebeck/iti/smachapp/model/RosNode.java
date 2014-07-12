package de.uni_luebeck.iti.smachapp.model;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Subscriber;

import java.net.URI;

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

    public RosNode(DebugModel model,String ip){
        this.model=model;
        this.ip=ip;
        executer=DefaultNodeMainExecutor.newDefault();
    }

    @Override
    public GraphName getDefaultNodeName() {
        return null;
    }

    @Override
    public void onStart(final ConnectedNode connectedNode){
        for(ISmachableSensor sen:model.getEditor().getRobot().getIntSensors()){
            Subscriber<IR> sub=connectedNode.newSubscriber(sen.getTopic(),sen.getTopicType());

            sub.addMessageListener(new MessageListener<IR>() {
                @Override
                public void onNewMessage(IR m) {
                    model.updateIR(m.getIr());
                }
            });
        }

        for(ISmachableSensor sen:model.getEditor().getRobot().getColorSensors()){
            Subscriber<Color_sensors> sub=connectedNode.newSubscriber(sen.getTopic(),sen.getTopicType());

            sub.addMessageListener(new MessageListener<Color_sensors>() {
                @Override
                public void onNewMessage(Color_sensors color_sensors) {
                    model.updateGroundColors(color_sensors.getSensors());
                }
            });
        }

        Subscriber<SmachContainerStatus> sub=connectedNode.newSubscriber("/Beep_State_Server/smach/container_status",SmachContainerStatus._TYPE);

        sub.addMessageListener(new MessageListener<SmachContainerStatus>() {
            @Override
            public void onNewMessage(SmachContainerStatus smachContainerStatus) {
                if(smachContainerStatus.getActiveStates().size()==1){
                    model.updateCurrentState(smachContainerStatus.getActiveStates().get(0));
                }else{
                    System.out.println("More then one State Active!");
                }
            }
        });
    }

    public void startNode(){
        URI muri=URI.create(ip);
        NodeConfiguration config=NodeConfiguration.newPublic("127.0.0.1",muri);
        config.setNodeName("SmachAppNode");
        executer.execute(this,config);
    }

    public void stopNode(){
        executer.shutdown();
    }
}
