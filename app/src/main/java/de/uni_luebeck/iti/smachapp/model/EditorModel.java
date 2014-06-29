package de.uni_luebeck.iti.smachapp.model;

import java.io.File;

import de.uni_luebeck.iti.smachapp.model.undo.UndoManager;

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
    private File saveFile;

    private UndoManager undoManager = new UndoManager();

    public EditorModel(String name) {
        stateMachine = new StateMachine(name);
        updateFileNames();
    }

    public void updateFileNames() {
        File path = XMLSaverLoader.PATH;
        pythonFile = new File(path, stateMachine.getName() + ".py");
        saveFile = new File(path, stateMachine.getName() + ".smach");
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

    public int getStateNameCounter() {
        return stateNameCounter;
    }

    public String getNextStateName() {
        String res = "S" + stateNameCounter;
        stateNameCounter++;
        return res;
    }

    public void setTransitionNameCounter(int i) {
        transitionNameCounter = i;
    }

    public int getTransitionNameCounter() {
        return transitionNameCounter;
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

    public File getSaveFile() {
        return saveFile;
    }

    public UndoManager getUndoManager() {
        return undoManager;
    }
}
