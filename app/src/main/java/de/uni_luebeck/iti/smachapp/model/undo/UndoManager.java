package de.uni_luebeck.iti.smachapp.model.undo;

import java.util.LinkedList;
import java.util.List;
import java.util.Stack;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class UndoManager {

    private List<UndoListener> listeners = new LinkedList<UndoListener>();

    private Stack<UndoableAction> undos = new Stack<UndoableAction>();
    private Stack<UndoableAction> redos = new Stack<UndoableAction>();

    public boolean addListener(UndoListener list) {
        return listeners.add(list);
    }

    public boolean removeListener(UndoListener list) {
        return listeners.remove(list);
    }

    protected void triggerListeners() {
        for (UndoListener list : listeners) {
            list.operationPerformed(this);
        }
    }

    public boolean isUndoAvailable() {
        return !undos.empty();
    }

    public boolean isRedoAvailable() {
        return !redos.empty();
    }

    public void newAction(UndoableAction action) {
        undos.push(action);
        redos.clear();
        triggerListeners();
    }

    public void undo() {
        if (isUndoAvailable()) {
            UndoableAction action = undos.pop();
            action.undo();
            redos.push(action);
            triggerListeners();
        }
    }

    public void redo() {
        if (isRedoAvailable()) {
            UndoableAction action = redos.pop();
            action.redo();
            undos.push(action);
            triggerListeners();
        }
    }

}