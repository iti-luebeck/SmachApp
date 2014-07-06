package de.uni_luebeck.iti.smachapp.model;

import java.util.LinkedList;
import java.util.List;

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

    public DebugModel(EditorModel model) {
        this.model = model;

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

    public void updateGroundColors(int[] colors) {
        if (colors.length != 3) {
            throw new IllegalArgumentException();
        }
        this.groundColors = colors;
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
}
