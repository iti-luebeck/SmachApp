package de.uni_luebeck.iti.smachapp.model;

import android.os.Environment;

import java.io.File;

/**
 * Created by Morten Mey on 27.04.2014.
 */
public class EditorModel {

    public enum EditorState {
        EDIT_STATES,
        EDIT_TRANSITIONS
    }

    private EditorState currentState = EditorState.EDIT_STATES;

    private StateMachine stateMachine;

    private int stateNameCounter = 0;
    private int transitionNameCounter = 0;

    private BeepRobot robot = new BeepRobot();

    private File pythonFile;

    public EditorModel(String name) {
        stateMachine = new StateMachine(name);
        File path = Environment.getExternalStorageDirectory();
        pythonFile = new File(path, stateMachine.getName() + ".py");
    }

    public void setCurrentState(EditorState newState) {
        currentState = newState;
    }

    public EditorState getCurrentState() {
        return currentState;
    }

    public StateMachine getStateMachine() {
        return stateMachine;
    }

    public void setStateNameCounter(int i) {
        stateNameCounter = i;
    }

    public String getNextStateName() {
        String res = "S" + stateNameCounter;
        stateNameCounter++;
        return res;
    }

    public void setTransitionNameCounter(int i) {
        transitionNameCounter = i;
    }

    public String getNextTransitionName() {
        String res = "T" + transitionNameCounter;
        transitionNameCounter++;
        return res;
    }

    public BeepRobot getRobot() {
        return robot;
    }

    public File getPythonFile() {
        return pythonFile;
    }
}
