package de.uni_luebeck.iti.smachapp.controller;

import android.graphics.PointF;
import android.graphics.RectF;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.widget.Toast;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.model.undo.AddState;
import de.uni_luebeck.iti.smachapp.model.undo.ChangeInitialState;
import de.uni_luebeck.iti.smachapp.model.undo.DeleteStates;
import de.uni_luebeck.iti.smachapp.model.undo.DragState;
import de.uni_luebeck.iti.smachapp.model.undo.StatePropertyChanged;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;
import de.uni_luebeck.iti.smachapp.utils.RectUtils;

/**
 * A controller for handling input, when the user is in state editing mode.
 * Created by Morten Mey on 28.04.2014.
 */
public class StateController implements ExtendedGestureListener {

    public static final float MARGIN = 25f;

    private StateMachineEditorController cont;

    private State dragged = null;
    private PointF originalPoint = new PointF();
    private PointF lastPoint = new PointF();
    private List<Transition> incomingTransitions;

    private RectF rect = new RectF();
    private PointF point = new PointF();

    private boolean isScroll = false;

    private boolean isActionModeActive = false;
    private List<State> selected = new LinkedList<State>();

    public StateController(StateMachineEditorController cont) {
        this.cont = cont;
    }

    private DragState dragAction = null;
    private StatePropertyChanged propertyChanged = null;

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        dragged = null;
        isScroll = false;
        cont.getView().highlighteStates(selected);

        point.set(motionEvent.getX(), motionEvent.getY());
        cont.getView().translatePoint(point);

        for (State s : cont.getModel().getStateMachine()) {
            cont.getView().getStateRect(s, rect);
            RectUtils.extendRect(rect,MARGIN);
            if (rect.contains(point.x, point.y)) {
                dragged = s;
                originalPoint.set(dragged.getX(), dragged.getY());
                lastPoint.set(originalPoint);
                incomingTransitions = cont.getModel().getStateMachine().getIncomingTransitions(dragged);
                dragAction = new DragState(dragged, incomingTransitions);
                break;
            }
        }
        return dragged != null;
    }

    @Override
    public void onUp(MotionEvent e) {
        if (isScroll) {
            for (Transition trans : dragged) {
                trans.getPath().fixEnd();
            }
            for (Transition trans : incomingTransitions) {
                trans.getPath().fixBeginning();
            }
            dragAction.operationDone();
            cont.getModel().getUndoManager().newAction(dragAction);
        }
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        if(!isActionModeActive) {
            selected.add(dragged);
            cont.getView().postInvalidate();
        }
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {

        point.x = motionEvent.getX();
        point.y = motionEvent.getY();
        cont.getView().translatePoint(point);

        if (isActionModeActive) {

            if(dragged!=null){
                if(selected.contains(dragged)){
                    selected.remove(dragged);
                }else{
                    selected.add(dragged);
                }

                isActionModeActive=cont.updateSelection(selected.size());
                cont.getView().postInvalidate();
            }

            return true;
        }

        if (dragged != null) {
            propertyChanged = new StatePropertyChanged(dragged, cont.getModel().getStateMachine());
            cont.showStateProperties(dragged);
            return true;
        }

        State s = new State(cont.getModel().getNextStateName(), point.x, point.y);
        AddState action = new AddState(s, cont.getModel().getStateMachine());
        action.redo();
        cont.getModel().getUndoManager().newAction(action);
        cont.getView().postInvalidate();

        return true;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        if (isActionModeActive) {
            return false;
        }
        if (dragged != null) {
            isScroll = true;
            point.x = motionEvent2.getX();
            point.y = motionEvent2.getY();
            cont.getView().translatePoint(point);

            dragged.setCenter(point.x, point.y);

            PointF temp = PointUtils.calculateDirection(lastPoint, point);
            lastPoint.set(point);

            for (Transition trans : dragged) {
                trans.getPath().moveKnots(temp.x, temp.y, 0);
            }
            for (Transition trans : incomingTransitions) {
                trans.getPath().moveKnots(temp.x, temp.y, -1);
            }

            cont.getView().postInvalidate();
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {

        point.x = motionEvent.getX();
        point.y = motionEvent.getY();
        cont.getView().translatePoint(point);

        if(isActionModeActive){

            if(dragged!=null){
                if(selected.contains(dragged)){
                    selected.remove(dragged);
                }else{
                    selected.add(dragged);
                }

                isActionModeActive=cont.updateSelection(selected.size());
                cont.getView().postInvalidate();
            }

            return;
        }
        if (dragged != null) {
            cont.showContextMenu();
            isActionModeActive = true;
            if (!selected.contains(dragged)) {
                selected.add(dragged);
            }
            dragged = null;
            cont.getView().postInvalidate();
        }else{
            State s = new State(cont.getModel().getNextStateName(), point.x, point.y);
            AddState action = new AddState(s, cont.getModel().getStateMachine());
            action.redo();
            cont.getModel().getUndoManager().newAction(action);
            dragged=s;
            isActionModeActive=true;
            selected.clear();
            selected.add(s);
            cont.showContextMenu();
            cont.getView().postInvalidate();
        }
    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }


    @Override
    public boolean onContextItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.context_menu_delete:
                for (Iterator<State> iter = selected.iterator(); iter.hasNext(); ) {
                    State s = iter.next();
                    if (s.isInitialState()) {
                        Toast toast = Toast.makeText(cont.getView().getContext(), R.string.cant_delete_initial_state, Toast.LENGTH_LONG);
                        toast.show();
                        iter.remove();
                    }
                }

                if (!selected.isEmpty()) {
                    DeleteStates deleteAction = new DeleteStates(selected, cont.getModel().getStateMachine());
                    deleteAction.redo();
                    cont.getModel().getUndoManager().newAction(deleteAction);
                }

                return true;

            case R.id.context_menu_properties:
                if (selected.size() == 1) {
                    propertyChanged = new StatePropertyChanged(selected.get(0), cont.getModel().getStateMachine());
                    cont.showStateProperties(selected.get(0));
                }
                return true;

            case R.id.context_menu_make_initial:
                if (selected.size() == 1) {
                    State oldIinitialState = null;
                    for (State x : cont.getModel().getStateMachine()) {
                        if (x.isInitialState()) {
                            oldIinitialState = x;
                            break;
                        }
                    }

                    if (oldIinitialState == selected.get(0)) {
                        return true;
                    }

                    ChangeInitialState changeAction = new ChangeInitialState(oldIinitialState, selected.get(0), cont.getModel().getStateMachine());
                    changeAction.redo();
                    cont.getModel().getUndoManager().newAction(changeAction);

                }
                return true;

            default:
                return false;
        }
    }

    @Override
    public void resumed() {
        if (propertyChanged != null) {

            if (propertyChanged.operationComplete()) {
                cont.getModel().getUndoManager().newAction(propertyChanged);
                cont.getModel().getStateMachine().fixTransitionEnds(dragged);
            }

            dragged = null;
            propertyChanged = null;
            selected.clear();
        }
    }

    @Override
    public void actionModeFinished() {
        isActionModeActive = false;
        selected.clear();
        cont.getView().postInvalidate();
    }
}
