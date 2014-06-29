package de.uni_luebeck.iti.smachapp.model.undo;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public interface UndoListener {

    public void operationPerformed(UndoManager manager);
}
