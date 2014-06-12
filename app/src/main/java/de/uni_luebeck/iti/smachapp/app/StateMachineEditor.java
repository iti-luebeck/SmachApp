package de.uni_luebeck.iti.smachapp.app;

import android.app.ActionBar;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.text.InputType;
import android.view.ContextMenu;
import android.view.KeyEvent;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;
import android.widget.TextView;

import de.uni_luebeck.iti.smachapp.controller.StateMachineEditorController;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.view.StateMachineView;

public class StateMachineEditor extends Activity {

    private static final String USE_OLD_MODEL="USE_OLD_EDITOR_MODEL";
    private static EditorModel oldModel=null;

    private StateMachineEditorController controller;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        EditorModel model;

        if(savedInstanceState!=null && savedInstanceState.getBoolean(USE_OLD_MODEL,false) && oldModel!=null){
            model=oldModel;
            oldModel=null;
        }else{
            model=new EditorModel(getIntent().getStringExtra("name"));
            oldModel=null;

            State state=new State(model.getNextStateName(),0,0,true);
            model.getStateMachine().addState(state);
        }

        setContentView(R.layout.activity_state_machine_editor);

        StateMachineView view=(StateMachineView)findViewById(R.id.editorView);
        registerForContextMenu(view);

        controller=new StateMachineEditorController(model,view,this);
    }

    @Override
    protected void onResume(){
        super.onResume();
        ActionBar bar=getActionBar();
        findViewById(R.id.editorView).postInvalidate();
        if(bar!=null){
            bar.hide();
        }
    }

    @Override
    protected void onSaveInstanceState(Bundle bundle){
        super.onSaveInstanceState(bundle);
        oldModel=controller.getModel();
        bundle.putBoolean(USE_OLD_MODEL,true);
    }

    public void switchModes(View view){
        if(view.getId()==R.id.edit_states){
            view.setEnabled(false);
            controller.modeSwitch(EditorModel.EditorState.EDIT_STATES);
            findViewById(R.id.edit_transitions).setEnabled(true);
        }else if(view.getId()==R.id.edit_transitions){
            view.setEnabled(false);
            controller.modeSwitch(EditorModel.EditorState.EDIT_TRANSITIONS);
            findViewById(R.id.edit_states).setEnabled(true);
        }
    }

    public void resetView(View view){
        controller.resetView();
    }

    @Override
    public void onCreateContextMenu(ContextMenu menu, View v, ContextMenu.ContextMenuInfo menuInfo) {
        super.onCreateContextMenu(menu,v,menuInfo);
        MenuInflater inflater=getMenuInflater();
        switch(controller.getModel().getCurrentState()){

            case EDIT_STATES:
                inflater.inflate(R.menu.context_menu_state,menu);
                break;
            case EDIT_TRANSITIONS:
                inflater.inflate(R.menu.context_menu_trans,menu);
                break;
        }
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        return controller.onContextItemSelected(item);
    }

    public void showStateProperties(State s){
        Intent intent=new Intent(this,StateProperty.class);
        StateProperty.setupState(s, controller.getModel().getRobot());
        startActivity(intent);
    }

    public void showTransitionProperites(Transition t) {
        Intent intent =new Intent(this,TransitionProperty.class);
        TransitionProperty.setupTransition(t,controller.getModel().getRobot());
        startActivity(intent);
    }

    public void transmit(View view){
        controller.compile();

        AlertDialog.Builder builder=new AlertDialog.Builder(this);

        builder.setTitle(R.string.enterAddress);
        final EditText text=new EditText(this);
        text.setInputType(InputType.TYPE_CLASS_PHONE);
        text.setImeOptions(EditorInfo.IME_ACTION_SEND);
        builder.setView(text);

        builder.setPositiveButton(R.string.send,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                controller.transmit(text.getText().toString());
            }
        });

        builder.setNegativeButton(R.string.disrecard,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
            }
        });

        final AlertDialog dia= builder.show();
        text.postDelayed(new Runnable() {
            @Override
            public void run() {
                InputMethodManager keyboard = (InputMethodManager)
                        getSystemService(Context.INPUT_METHOD_SERVICE);
                keyboard.showSoftInput(text, 0);
            }
        }, 200);

        text.setOnEditorActionListener(new TextView.OnEditorActionListener() {
            @Override
            public boolean onEditorAction(TextView textView, int i, KeyEvent keyEvent) {
                dia.dismiss();
                controller.transmit(text.getText().toString());
                return true;
            }
        });

    }

}
