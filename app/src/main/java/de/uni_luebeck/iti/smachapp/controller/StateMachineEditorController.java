package de.uni_luebeck.iti.smachapp.controller;

import android.gesture.GestureOverlayView;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;

import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class StateMachineEditorController implements ITouchController,ScaleGestureDetector.OnScaleGestureListener,GestureDetector.OnGestureListener {

    private EditorModel model;
    private StateMachineView view;
    private ExtendedGestureListener subController;

    private GestureDetector detector;
    private ScaleGestureDetector scaleDetector;

    private boolean isMulitTouch=false;

    public StateMachineEditorController(EditorModel model,StateMachineView view){
        this.model=model;
        this.view=view;
        view.setModel(model);
        view.setController(this);
        detector=new GestureDetector(view.getContext(),this);
        scaleDetector=new ScaleGestureDetector(view.getContext(),this);

        switch (model.getCurrentState()) {
            case EDIT_STATES:
                subController=new StateController(this);
                break;
            case EDIT_TRANSITIONS:
                subController=new TransitionController(this);
                break;
        }
    }

    public EditorModel getModel(){
        return model;
    }

    public StateMachineView getView(){
        return view;
    }

    public void modeSwitch(EditorModel.EditorState state){
        model.setCurrentState(state);

        switch (state) {
            case EDIT_STATES:
                subController=new StateController(this);
                break;
            case EDIT_TRANSITIONS:
                subController=new TransitionController(this);
                break;
        }
    }

    public void resetView(){
        view.reset();
    }

    @Override
    public boolean onTouchEvent(MotionEvent e) {
        switch(e.getActionMasked()){
            case MotionEvent.ACTION_DOWN:
                isMulitTouch=false;
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                isMulitTouch=true;
                break;

            case MotionEvent.ACTION_UP:
                subController.onUp(e);
        }

        scaleDetector.onTouchEvent(e);
        detector.onTouchEvent(e);
        return true;
    }

    @Override
    public boolean onDown(MotionEvent motionEvent) {
        subController.onDown(motionEvent);
        return true;
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
    }

    @Override
    public boolean onSingleTapUp(MotionEvent motionEvent) {
        return subController.onSingleTapUp(motionEvent);
    }

    @Override
    public boolean onScroll(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        boolean handled=false;

        if(!isMulitTouch){
            handled=subController.onScroll(motionEvent,motionEvent2,v,v2);
        }

        if(!handled){
            view.translate(v,v2);
        }

        return true;
    }

    @Override
    public void onLongPress(MotionEvent motionEvent) {

    }

    @Override
    public boolean onFling(MotionEvent motionEvent, MotionEvent motionEvent2, float v, float v2) {
        return false;
    }

    @Override
    public boolean onScale(ScaleGestureDetector scaleGestureDetector) {
        view.scale(scaleGestureDetector.getScaleFactor());
        return true;
    }

    @Override
    public boolean onScaleBegin(ScaleGestureDetector scaleGestureDetector) {
        return true;
    }

    @Override
    public void onScaleEnd(ScaleGestureDetector scaleGestureDetector) {

    }
}
