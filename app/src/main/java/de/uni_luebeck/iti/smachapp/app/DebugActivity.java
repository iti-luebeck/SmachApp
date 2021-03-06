/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of L�beck
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
package de.uni_luebeck.iti.smachapp.app;

import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.FragmentTransaction;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import java.util.ArrayList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.controller.DebugController;
import de.uni_luebeck.iti.smachapp.model.DebugModel;
import de.uni_luebeck.iti.smachapp.model.DebugModelObserver;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.view.SensorView;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class DebugActivity extends Activity implements DebugModelObserver {

    private static String TAB_INDEX = "TAB_INDEX";
    private static String USE_OLD_MODEL = "USE_OLD_MODEL";

    private StateMachineView smView;
    private SensorView sensorView;

    private DebugController controller;

    private DebugModel model;
    private static EditorModel setupModel;
    private static DebugModel oldModel;

    private ViewGroup group;

    private List<State> currentState = new ArrayList<State>(1);
    private List<Transition> lastTransition = new ArrayList<Transition>(1);

    private AlertDialog dialog;
    private boolean isDestroyed=false;

    public static void setup(EditorModel model) {
        setupModel = model;
    }

    @Override
    protected void onCreate(Bundle bundle) {
        super.onCreate(bundle);

        boolean running;

        if (bundle != null && bundle.getBoolean(USE_OLD_MODEL)) {
            this.model = oldModel;
            running=true;
        } else {
            this.model = new DebugModel(setupModel,getIntent().getStringExtra(StateMachineEditor.IP));
            running=false;
        }
        model.addObserver(this);

        controller = new DebugController(model, this);

        group = new LinearLayout(this);
        group.setOnTouchListener(controller);

        smView = new StateMachineView(this);
        sensorView = new SensorView(this);
        smView.setModel(model.getEditor());
        currentState.add(model.getCurrentState());
        smView.highlighteTransitions(lastTransition);
        smView.highlighteStates(currentState);
        sensorView.setModel(model);

        controller.setView(smView);

        AlertDialog.Builder builder=new AlertDialog.Builder(this);
        builder.setView(View.inflate(this, R.layout.working_dialog, null));
        builder.setCancelable(false);
        dialog=builder.create();

        final Activity activity = this;

        ActionBar.TabListener listener = new ActionBar.TabListener() {
            @Override
            public void onTabSelected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
                switch (tab.getPosition()) {
                    case 0:
                        group.addView(smView);
                        break;
                    case 1:
                        group.addView(sensorView);
                        break;
                }
                activity.invalidateOptionsMenu();
            }

            @Override
            public void onTabUnselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
                switch (tab.getPosition()) {
                    case 0:
                        group.removeAllViews();
                        break;
                    case 1:
                        group.removeAllViews();
                        break;
                }
            }

            @Override
            public void onTabReselected(ActionBar.Tab tab, FragmentTransaction fragmentTransaction) {
            }
        };

        ActionBar bar = getActionBar();
        bar.setNavigationMode(ActionBar.NAVIGATION_MODE_TABS);
        ActionBar.Tab tab = bar.newTab().setTabListener(listener).setText(R.string.debug_automat);
        bar.addTab(tab);
        tab = bar.newTab().setTabListener(listener).setText(R.string.sensor_data);
        bar.addTab(tab);

        if (running) {
            bar.setSelectedNavigationItem(bundle.getInt(TAB_INDEX));
        }else{
            controller.play();
            invalidateOptionsMenu();
        }

        setContentView(group);
    }


    @Override
    protected void onPause(){
        super.onPause();
        hideWorkingDialog();
    }

    @Override
    protected void onDestroy(){
        hideWorkingDialog();
        isDestroyed=true;
        super.onDestroy();
        System.out.println("Destroying");
        controller.stop(true);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu items for use in the action bar
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.debug_action_bar, menu);
        return super.onCreateOptionsMenu(menu);
    }

    @Override
    public boolean onPrepareOptionsMenu(Menu menu) {
        if (getNavIndex() == 1) {
            menu.findItem(R.id.action_center).setVisible(false);
        }
        if(!model.isRunning()){
            MenuItem item=menu.findItem(R.id.action_stop);

            item.setIcon(R.drawable.ic_action_play);
            item.setTitle(R.string.transmitButton);
        }
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.action_stop:
                if(model.isRunning()){
                    controller.stop(false);
                }else{
                    controller.play();
                }

                break;
            case R.id.action_center:
                smView.reset();
                break;
        }
        return true;
    }

    @Override
    protected void onSaveInstanceState(Bundle bundle) {
        super.onSaveInstanceState(bundle);
        oldModel = model;
        bundle.putBoolean(USE_OLD_MODEL, true);
        bundle.putInt(TAB_INDEX, getActionBar().getSelectedNavigationIndex());
    }

    public int getNavIndex() {
        return getActionBar().getSelectedNavigationIndex();
    }

    public void setNavIndex(int i) {
        getActionBar().setSelectedNavigationItem(i);
    }

    @Override
    public void onModelChange(DebugModel model) {
        currentState.clear();
        lastTransition.clear();
        currentState.add(model.getCurrentState());
        lastTransition.addAll(model.getLastTransitions());
        smView.postInvalidate();
    }

    public void showWorkingDialog(){
        if(!isDestroyed) {
            dialog.show();
        }
    }

    public void hideWorkingDialog(){
        try {
            dialog.dismiss();
        }catch (Exception ex){
            ex.printStackTrace();
        }
    }
}
