package de.uni_luebeck.iti.smachapp.controller;

import android.gesture.GestureOverlayView;
import android.view.ContextMenu;
import android.view.GestureDetector;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import de.uni_luebeck.iti.smachapp.app.StateMachineEditor;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class StateMachineEditorController implements View.OnTouchListener,ScaleGestureDetector.OnScaleGestureListener,GestureDetector.OnGestureListener {

    private EditorModel model;
    private StateMachineView view;
    private StateMachineEditor activity;
    private ExtendedGestureListener subController;

    private GestureDetector detector;
    private ScaleGestureDetector scaleDetector;

    private boolean isMulitTouch=false;

    public StateMachineEditorController(EditorModel model,StateMachineView view,StateMachineEditor activity){
        this.model=model;
        this.view=view;
        this.activity=activity;
        view.setModel(model);
        view.setOnTouchListener(this);
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
    public boolean onDown(MotionEvent motionEvent) {
        subController.onDown(motionEvent);
        return true;
    }

    @Override
    public void onShowPress(MotionEvent motionEvent) {
        subController.onShowPress(motionEvent);
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
        subController.onLongPress(motionEvent);
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

    public boolean onContextItemSelected(MenuItem item) {
        return subController.onContextItemSelected(item);
    }

    public void showContextMenu(){
        activity.openContextMenu(view);
    }

    @Override
    public boolean onTouch(View view, MotionEvent e) {
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
}
