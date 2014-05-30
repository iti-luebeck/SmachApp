package de.uni_luebeck.iti.smachapp.controller;

import android.graphics.PointF;
import android.graphics.RectF;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.widget.Toast;

import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * A controller for handling input, when the user is in state editing mode.
 * Created by Morten Mey on 28.04.2014.
 */
public class StateController implements ExtendedGestureListener{

    private StateMachineEditorController cont;

    private State dragged=null;
    private PointF originalPoint=new PointF();
    private PointF lastPoint=new PointF();
    private List<Transition> incomingTransitions;

    private RectF rect=new RectF();
    private PointF point=new PointF();

    private boolean upHandled =false;

    public StateController(StateMachineEditorController cont){
        this.cont=cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        dragged=null;
        upHandled =false;

        point.set(motionEvent.getX(),motionEvent.getY());
        cont.getView().translatePoint(point);

        for(State s:cont.getModel().getStateMachine()){
            cont.getView().getStateRect(s,rect);
            if(rect.contains(point.x,point.y)){
                dragged=s;
                originalPoint.set(dragged.getX(),dragged.getY());
                lastPoint.set(originalPoint);
                incomingTransitions=cont.getModel().getStateMachine().getIncomingTransitions(dragged);
                break;
            }
        }
        return dragged!=null;
    }

    @Override
    public void onUp(MotionEvent e) {
        if(!upHandled) {
            cont.getView().highlighteState(null);
        }

        if(dragged!=null){
            for(Transition trans:dragged){
                trans.getPath().fixEnd();
            }
            for(Transition trans:incomingTransitions){
                trans.getPath().fixBeginning();
            }
        }
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        cont.getView().highlighteState(dragged);
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        upHandled=true;

        if(dragged!=null){
            cont.showStateProperties(dragged);
            return true;
        }

        point.x=motionEvent.getX();
        point.y=motionEvent.getY();
        cont.getView().translatePoint(point);
        State s=new State(cont.getModel().getNextStateName(),point.x,point.y);
        cont.getModel().getStateMachine().addState(s);
        cont.getView().postInvalidate();

        return true;
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        if(dragged!=null){
            point.x=motionEvent2.getX();
            point.y=motionEvent2.getY();
            cont.getView().translatePoint(point);

            dragged.setCenter(point.x,point.y);

            PointF temp= PointUtils.calculateDirection(lastPoint,point);
            lastPoint.set(point);

            for(Transition trans:dragged){
                trans.getPath().moveKnots(temp.x,temp.y,0);
            }
            for(Transition trans:incomingTransitions){
                trans.getPath().moveKnots(temp.x,temp.y,-1);
            }

            cont.getView().postInvalidate();
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {
        if(dragged!=null) {
            cont.showContextMenu();
            upHandled = true;
        }
    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }


    @Override
    public boolean onContextItemSelected(MenuItem item) {
        cont.getView().highlighteState(null);

        switch(item.getItemId()){
            case R.id.context_menu_delete:
                if(dragged.isInitialState()){
                    Toast toast= Toast.makeText(cont.getView().getContext(),R.string.cant_delete_initial_state,Toast.LENGTH_LONG);
                    toast.show();
                }else {
                    cont.getModel().getStateMachine().removeState(dragged);
                }

                return true;

            case R.id.context_menu_properties:
                cont.showStateProperties(dragged);
                return true;

            case R.id.context_menu_make_initial:
                for(State x:cont.getModel().getStateMachine()){
                    if(x.isInitialState()){
                        x.setInitialState(false);
                    }
                }

                dragged.setInitialState(true);
                return true;

            default:
                return false;
        }
    }
}
