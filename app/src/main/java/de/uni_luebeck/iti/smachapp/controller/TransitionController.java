package de.uni_luebeck.iti.smachapp.controller;

import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.RectF;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.widget.Toast;

import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.BezierPath;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.model.undo.AddTransition;
import de.uni_luebeck.iti.smachapp.model.undo.DeleteTransitions;
import de.uni_luebeck.iti.smachapp.model.undo.DragTransition;
import de.uni_luebeck.iti.smachapp.model.undo.ResetTransitions;
import de.uni_luebeck.iti.smachapp.model.undo.TransitionPropertyChanged;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;
import de.uni_luebeck.iti.smachapp.utils.RectUtils;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class TransitionController implements ExtendedGestureListener {

    public static final float MARGIN = 25f;

    private StateMachineEditorController cont;

    private Path tempPath = new Path();
    private List<PointF> points = new LinkedList<PointF>();

    private boolean isLongPress = false;
    private boolean isScroll = false;

    private State begin = null;
    private Transition selected = null;
    private int closestPoint = 0;
    private PointF lastPoint = new PointF();

    private boolean isActionModeActive = false;
    private List<Transition> selectedTransition = new LinkedList<Transition>();

    private DragTransition dragAction = null;
    private TransitionPropertyChanged propertyAction = null;

    public TransitionController(StateMachineEditorController cont) {
        this.cont = cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        begin = null;
        selected = null;
        tempPath.reset();
        points.clear();
        isLongPress = false;
        isScroll = false;
        cont.getView().highlighteTransitions(selectedTransition);

        if (!isActionModeActive) {
            selectedTransition.clear();
        }


        PointF point = new PointF(motionEvent.getX(), motionEvent.getY());
        cont.getView().translatePoint(point);

        RectF rect = new RectF();
        for (State s : cont.getModel().getStateMachine()) {
            cont.getView().getStateRect(s, rect);
            if (rect.contains(point.x, point.y)) {
                begin = s;
                tempPath.moveTo(point.x, point.y);
                points.add(new PointF(point.x, point.y));
                cont.getView().setTempPath(tempPath);
                return true;
            }
        }

        findPickedTransition(point);

        if(selected!=null){
            dragAction = new DragTransition(selected);
        }

        return selected != null;
    }

    @Override
    public void onUp(MotionEvent e) {
        cont.getView().setTempPath(null);
        if (isLongPress) {
            return;
        }

        if (begin != null && isScroll) {
            State end = null;
            PointF point = new PointF(e.getX(), e.getY());
            cont.getView().translatePoint(point);
            RectF rect = new RectF();
            for (State s : cont.getModel().getStateMachine()) {
                cont.getView().getStateRect(s, rect);
                RectUtils.extendRect(rect, MARGIN);
                if (rect.contains(point.x, point.y)) {
                    end = s;
                    points.add(new PointF(point.x, point.y));
                    break;
                }
            }
            if (end == null) {
                Toast toast = Toast.makeText(cont.getView().getContext(), R.string.must_end_at_state, Toast.LENGTH_LONG);
                toast.show();
            } else if (begin == end) {
                Toast toast = Toast.makeText(cont.getView().getContext(), R.string.begin_must_not_be_end, Toast.LENGTH_LONG);
                toast.show();
            } else {
                cont.getView().getStateRect(begin, rect);
                BezierPath.removePointsInOval(rect, points);
                BezierPath.moveOnOval(rect, points.get(0));
                cont.getView().getStateRect(end, rect);
                BezierPath.removePointsInOval(rect, points);
                BezierPath.moveOnOval(rect, points.get(points.size() - 1));

                BezierPath.filterPoints(points);

                Transition newTrans = new Transition(begin, end, cont.getModel().getNextTransitionName(), new BezierPath(points));
                AddTransition addAction = new AddTransition(newTrans, cont.getModel().getStateMachine());
                addAction.redo();
                cont.getModel().getUndoManager().newAction(addAction);
            }
        } else if (selected != null && isScroll) {

            boolean connected = false;
            RectF rect = new RectF();

            if (closestPoint == 0) {
                for (State s : cont.getModel().getStateMachine()) {
                    cont.getView().getStateRect(s, rect);
                    RectUtils.extendRect(rect, MARGIN);
                    PointF p = selected.getPath().getPoints().get(0);
                    if (rect.contains(p.x, p.y)) {
                        connected = true;
                        if (selected.getPreviousState() != null) {
                            selected.getPreviousState().removeTransition(selected);
                        }
                        selected.setPreviousState(s);
                        s.addTransition(selected);
                        RectUtils.extendRect(rect, -MARGIN);
                        BezierPath.moveOnOval(rect, p);
                        break;
                    }
                }
                if (!connected) {
                    dragAction.undo();
                    dragAction = null;
                    Toast toast = Toast.makeText(cont.getView().getContext(), R.string.unconnected_transition, Toast.LENGTH_LONG);
                    toast.show();
                    return;
                }

            } else if (closestPoint == selected.getPath().getPoints().size() - 1) {
                for (State s : cont.getModel().getStateMachine()) {
                    cont.getView().getStateRect(s, rect);
                    RectUtils.extendRect(rect, MARGIN);
                    PointF p = selected.getPath().getPoints().get(selected.getPath().getPoints().size() - 1);
                    if (rect.contains(p.x, p.y)) {
                        connected = true;
                        selected.setFollowerState(s);
                        RectUtils.extendRect(rect, -MARGIN);
                        BezierPath.moveOnOval(rect, p);
                        break;
                    }
                }
                if (!connected) {
                    dragAction.undo();
                    dragAction = null;
                    Toast toast = Toast.makeText(cont.getView().getContext(), R.string.unconnected_transition, Toast.LENGTH_LONG);
                    toast.show();
                    return;
                }
            }

            selected.getPath().fixBeginning();
            selected.getPath().fixEnd();

            dragAction.operationDone();
            cont.getModel().getUndoManager().newAction(dragAction);
            dragAction = null;
        }
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        selectedTransition.add(selected);
        cont.getView().postInvalidate();
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        if (isActionModeActive) {
            PointF point = new PointF(motionEvent.getX(), motionEvent.getY());
            cont.getView().translatePoint(point);

            findPickedTransition(point);

            if (selected == null) {
                return false;
            } else if (selectedTransition.contains(selected)) {
                selectedTransition.remove(selected);
            } else {
                selectedTransition.add(selected);
            }
            cont.getView().postInvalidate();
            cont.updateSelection(selectedTransition.size());
            return true;
        }
        if (selected != null) {
            propertyAction = new TransitionPropertyChanged(selected);
            cont.showTransitionProperties(selected);
            return true;
        }
        return false;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {

        if (isActionModeActive) {
            return false;
        }

        isScroll = true;
        PointF point = new PointF(motionEvent2.getX(), motionEvent2.getY());
        cont.getView().translatePoint(point);

        if (begin != null) {
            tempPath.lineTo(point.x, point.y);
            points.add(new PointF(point.x, point.y));
            cont.getView().postInvalidate();
            return true;
        } else if (selected != null) {

            PointF temp = PointUtils.calculateDirection(lastPoint, point);
            lastPoint.set(point);

            selected.getPath().moveKnots(temp.x, temp.y, closestPoint);
            cont.getView().postInvalidate();

            return true;
        } else {
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {
        if (selected != null) {
            isLongPress = true;
            cont.showContextMenu();
            isActionModeActive = true;
            if (!selectedTransition.contains(selected)) {
                selectedTransition.add(selected);
            }
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
                DeleteTransitions deleteAction = new DeleteTransitions(selectedTransition, cont.getModel().getStateMachine());
                deleteAction.redo();
                cont.getModel().getUndoManager().newAction(deleteAction);
                return true;

            case R.id.context_menu_reset:
                ResetTransitions resetAction = new ResetTransitions(selectedTransition);
                resetAction.redo();
                cont.getModel().getUndoManager().newAction(resetAction);
                return true;

            case R.id.context_menu_properties:
                if (selectedTransition.size() == 1) {
                    propertyAction = new TransitionPropertyChanged(selectedTransition.get(0));
                    cont.showTransitionProperties(selectedTransition.get(0));
                }
                return true;

            default:
                return false;
        }
    }

    @Override
    public void resumed() {
        if (propertyAction != null) {
            if (propertyAction.operationDone()) {
                cont.getModel().getUndoManager().newAction(propertyAction);
            }
            propertyAction = null;
        }
    }

    @Override
    public void actionModeFinished() {
        isActionModeActive = false;
        selectedTransition.clear();
        cont.getView().postInvalidate();
    }

    private void findPickedTransition(PointF point){
        selected=null;
        float minDistance = BezierPath.MIN_DISTANCE;
        for (Transition trans : cont.getModel().getStateMachine().getTransitions()) {
            List<PointF> transPoints = trans.getPath().getPoints();
            for (int i = 0; i < transPoints.size(); i++) {
                PointF curr = transPoints.get(i);
                float dist = PointUtils.distance(curr, point);

                if (dist < minDistance) {
                    minDistance = dist;
                    selected = trans;
                    lastPoint.set(point);

                    closestPoint = i;
                }
            }
        }

    }

}
