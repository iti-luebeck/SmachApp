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
        pythonFile = new File(path, stateMachine.getName() + XMLSaverLoader.PYTHON_FILE_ENDING);
        saveFile = new File(path, stateMachine.getName() + XMLSaverLoader.FILE_ENDING);
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


    public String getNextStateName() {
        int i=stateMachine.getStates().size();

        String res="S"+i;

        while(stateMachine.getState(res)!=null){
            i++;
            res="S"+i;
        }

        return res;
    }

    public String getNextTransitionName() {
        int i=stateMachine.getTransitions().size();

        String res="T"+i;

        while(stateMachine.getTransition(res)!=null){
            i++;
            res="T"+i;
        }

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
