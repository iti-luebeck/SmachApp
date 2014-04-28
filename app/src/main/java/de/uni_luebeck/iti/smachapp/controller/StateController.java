package de.uni_luebeck.iti.smachapp.controller;

import android.graphics.PointF;
import android.graphics.RectF;
import android.view.GestureDetector;
import android.view.MotionEvent;

import de.uni_luebeck.iti.smachapp.model.State;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class StateController implements ExtendedGestureListener{

    private StateMachineEditorController cont;
    private State dragged=null;
    private RectF rect=new RectF();
    private PointF point=new PointF();

    public StateController(StateMachineEditorController cont){
        this.cont=cont;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        dragged=null;
        for(State s:cont.getModel().getStateMachine()){
            cont.getView().getStateRect(s,rect);
            point.x=motionEvent.getX();
            point.y=motionEvent.getY();
            cont.getView().translatePoint(point);
            if(rect.contains(point.x,point.y)){
                dragged=s;
                break;
            }
        }
        return dragged!=null;
    }

    @Override
    public void onUp(MotionEvent e) {

    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {

    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        if(dragged!=null){
            return false;
        }

        point.x=motionEvent.getX();
        point.y=motionEvent.getY();
        cont.getView().translatePoint(point);
        State s=new State(cont.getModel().getNextName(),point.x,point.y);
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
            cont.getView().postInvalidate();
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {

    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }
}
