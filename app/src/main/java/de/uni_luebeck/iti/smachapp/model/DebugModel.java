package de.uni_luebeck.iti.smachapp.model;

import java.util.LinkedList;
import java.util.List;

import beep_msgs.Color;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class DebugModel {
    private EditorModel model;

    private State currentState;
    private Transition lastTransition=null;

    private short[] ir = new short[8];
    private int[] groundColors = new int[3];

    private List<DebugModelObserver> observers = new LinkedList<DebugModelObserver>();

    private boolean isRunning=false;
    private String ip;

    public DebugModel(EditorModel model,String ip) {
        this.model = model;
        this.ip=ip;

        for (State s : model.getStateMachine()) {
            if (s.isInitialState()) {
                currentState = s;
                break;
            }
        }
    }

    public EditorModel getEditor() {
        return model;
    }

    public void updateCurrentState(String state) {
        for(Transition t:currentState){
            if(t.getFollowerState().getName().equals(state)){
                lastTransition=t;
                currentState=t.getFollowerState();
                break;
            }
        }

        notifyObservers();
    }

    public void updateIR(short[] ir) {
        if (ir.length != 8) {
            throw new IllegalArgumentException();
        }
        this.ir = ir;
        notifyObservers();
    }

    public void updateGroundColors(List<Color> colors) {
        if (colors.size() != 3) {
            throw new IllegalArgumentException();
        }
        groundColors[0]=colorToInt(colors.get(0));
        groundColors[1]=colorToInt(colors.get(1));
        groundColors[2]=colorToInt(colors.get(2));
        notifyObservers();
    }

    private void notifyObservers() {
        for (DebugModelObserver obs : observers) {
            obs.onModelChange(this);
        }
    }

    public boolean addObserver(DebugModelObserver obs) {
        return observers.add(obs);
    }

    public boolean removeObserver(DebugModelObserver obs) {
        return observers.remove(obs);
    }

    public State getCurrentState() {
        return currentState;
    }

    public Transition getLastTransition(){
        return lastTransition;
    }

    public short[] getIr() {
        return ir;
    }

    public int[] getGroundColors() {
        return groundColors;
    }

    public boolean isRunning(){
        return isRunning;
    }

    public void setRunning(boolean running){
        isRunning=running;
    }

    public String getIp(){
        return ip;
    }

    private static int colorToInt(Color col){
        return android.graphics.Color.argb(col.getW(),col.getR(),col.getG(),col.getB());
    }
}
