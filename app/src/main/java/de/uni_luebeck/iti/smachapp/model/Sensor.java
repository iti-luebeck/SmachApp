package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.ISmachableSensor;
import de.uni_luebeck.iti.smachGenerator.Operator;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public abstract class Sensor implements ISmachableSensor {

    protected String topic;
    protected String key;
    protected String objectInMessage;
    protected String topicType;
    protected String topicPackage;

    public Sensor(String key,String topic,String topicType,String topicPackage,String objectInMessage){
        this.topic=topic;
        this.key=key;
        this.objectInMessage=objectInMessage;
        this.topicType=topicType;
        this.topicPackage=topicPackage;
    }

    @Override
    public String getTopic() {
        return topic;
    }

    @Override
    public String getKey() {
        return key;
    }

    @Override
    public String getObjectInMessage() {
        return objectInMessage;
    }

    @Override
    public String getTopicType() {
        return topicType;
    }

    @Override
    public String getTopicPackage() {
        return topicPackage;
    }
}
