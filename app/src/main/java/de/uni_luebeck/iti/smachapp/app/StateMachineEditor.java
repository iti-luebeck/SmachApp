package de.uni_luebeck.iti.smachapp.app;

import android.app.ActionBar;
import android.app.Activity;
import android.os.Bundle;
import android.view.ContextMenu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;

import de.uni_luebeck.iti.smachapp.controller.StateMachineEditorController;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
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
            model=new EditorModel();
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
        inflater.inflate(R.menu.context_menu,menu);
    }

    @Override
    public boolean onContextItemSelected(MenuItem item) {
        return controller.onContextItemSelected(item);
    }
}
