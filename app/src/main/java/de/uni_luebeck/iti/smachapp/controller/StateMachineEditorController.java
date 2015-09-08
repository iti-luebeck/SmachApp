/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package de.uni_luebeck.iti.smachapp.controller;

import android.content.Context;
import android.view.ActionMode;
import android.view.GestureDetector;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import java.io.IOException;

import de.uni_luebeck.iti.smachGenerator.SmachAutomat;
import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.app.StateMachineEditor;
import de.uni_luebeck.iti.smachapp.model.BeepRobot;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.model.XMLSaverLoader;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public class StateMachineEditorController implements View.OnTouchListener, ScaleGestureDetector.OnScaleGestureListener, GestureDetector.OnGestureListener {

    private EditorModel model;
    private StateMachineView view;
    private StateMachineEditor activity;
    private ExtendedGestureListener subController;

    private GestureDetector detector;
    private ScaleGestureDetector scaleDetector;

    private boolean isMulitTouch = false;
    private ActionMode actionMode = null;

    private boolean propertiesOpening=false;

    public StateMachineEditorController(EditorModel model, StateMachineView view, StateMachineEditor activity) {
        this.model = model;
        this.view = view;
        this.activity = activity;
        view.setModel(model);
        view.setOnTouchListener(this);
        detector = new GestureDetector(view.getContext(), this);
        scaleDetector = new ScaleGestureDetector(view.getContext(), this);

        switch (model.getCurrentState()) {
            case EDIT_STATES:
                subController = new StateController(this);
                break;
            case EDIT_TRANSITIONS:
                subController = new TransitionController(this);
                break;
        }
    }

    public EditorModel getModel() {
        return model;
    }

    public StateMachineView getView() {
        return view;
    }

    public void modeSwitch(EditorModel.EditorState state) {

        if (state != model.getCurrentState()) {

            if (actionMode != null) {
                actionMode.finish();
            }

            model.setCurrentState(state);

            switch (state) {
                case EDIT_STATES:
                    subController = new StateController(this);
                    break;
                case EDIT_TRANSITIONS:
                    subController = new TransitionController(this);
                    break;
            }
        }
    }

    public void resetView() {
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
        boolean handled = false;

        if (!isMulitTouch) {
            handled = subController.onScroll(motionEvent, motionEvent2, v, v2);
        }

        if (!handled) {
            view.translate(v, v2);
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

    public void showContextMenu() {
        actionMode = activity.startActionMode(new ActionMode.Callback() {
            @Override
            public boolean onCreateActionMode(ActionMode mode, Menu menu) {
                MenuInflater inflater = mode.getMenuInflater();
                inflater.inflate(R.menu.action_mode, menu);

                switch (model.getCurrentState()) {
                    case EDIT_STATES:
                        mode.getMenu().findItem(R.id.context_menu_reset).setVisible(false);
                        break;

                    case EDIT_TRANSITIONS:
                        mode.getMenu().findItem(R.id.context_menu_make_initial).setVisible(false);
                        break;
                }

                return true;

            }

            @Override
            public boolean onPrepareActionMode(ActionMode mode, Menu menu) {
                return false;
            }

            @Override
            public boolean onActionItemClicked(ActionMode mode, MenuItem menuItem) {
                boolean res = subController.onContextItemSelected(menuItem);
                actionMode.finish();
                return res;
            }

            @Override
            public void onDestroyActionMode(ActionMode mode) {
                subController.actionModeFinished();
                actionMode = null;
            }
        });
        actionMode.setTitle("1");
    }

    public boolean updateSelection(int selectionSize) {
        if (actionMode == null) {
            return false;
        }

        actionMode.setTitle(Integer.toString(selectionSize));

        if (selectionSize == 0) {
            actionMode.finish();
            return false;
        } else if (selectionSize > 1) {
            actionMode.getMenu().findItem(R.id.context_menu_make_initial).setVisible(false);
            actionMode.getMenu().findItem(R.id.context_menu_properties).setVisible(false);
            return true;

        } else {
            switch (model.getCurrentState()) {
                case EDIT_STATES:
                    actionMode.getMenu().findItem(R.id.context_menu_make_initial).setVisible(true);
                    //falls through
                case EDIT_TRANSITIONS:
                    actionMode.getMenu().findItem(R.id.context_menu_properties).setVisible(true);
                    break;
            }
            return true;
        }
    }

    @Override
    public boolean onTouch(View view, MotionEvent e) {
        switch (e.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                isMulitTouch = false;
                break;
            case MotionEvent.ACTION_POINTER_DOWN:
                isMulitTouch = true;
                break;

            case MotionEvent.ACTION_UP:
                subController.onUp(e);
        }

        scaleDetector.onTouchEvent(e);
        detector.onTouchEvent(e);
        return true;
    }

    public void showStateProperties(State s) {
        if(propertiesOpening){
            return;
        }

        propertiesOpening=true;
        activity.showStateProperties(s);
    }

    public void showTransitionProperties(Transition t, int priority, int maxPriority) {
        if(propertiesOpening){
            return;
        }

        propertiesOpening=true;
        activity.showTransitionProperites(t, priority, maxPriority);
    }

    public void resumed() {
        propertiesOpening=false;
        subController.resumed();
    }

    public void save() {
        try {
            XMLSaverLoader.save(model);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    public void compile() {
        BeepRobot robot = model.getRobot();
        XMLSaverLoader.PATH.mkdirs();
        SmachAutomat automat = new SmachAutomat(model.getStateMachine().getStates(), model.getRobot().getSensors(), model.getRobot().getActuators(), "zusmoro_state_machine");
        automat.saveToFile(model.getPythonFile());
    }

    public Context getContext(){
        return activity;
    }
}
