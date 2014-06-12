package de.uni_luebeck.iti.smachapp.model;

import de.uni_luebeck.iti.smachGenerator.ISmachableAction;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class Action implements ISmachableAction {

    private String key;
    private int value;

    public Action(String key, int value) {
        this.key = key;
        this.value = value;
    }

    @Override
    public String getKey() {
        return key;
    }

    @Override
    public int getValue() {
        return value;
    }

    public void setKey(String key) {
        this.key = key;
    }

    public void setValue(int value) {
        this.value = value;
    }
}
