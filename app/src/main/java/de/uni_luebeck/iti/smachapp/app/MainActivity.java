package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.text.InputType;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;
import android.widget.TextView;

public class MainActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }


    public void newFSM(View view){

        AlertDialog.Builder builder=new AlertDialog.Builder(this);

        builder.setTitle(R.string.enterMachineName);
        final EditText text=new EditText(this);
        text.setInputType(InputType.TYPE_CLASS_TEXT);
        text.setImeOptions(EditorInfo.IME_ACTION_NEXT);
        text.setText(R.string.newMachine);
        text.setSelection(text.getText().length());
        builder.setView(text);

        final Activity activity=this;

        builder.setPositiveButton(R.string.next,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                Intent intent=new Intent(activity, StateMachineEditor.class);
                intent.putExtra("name",text.getText().toString());
                startActivity(intent);
            }
        });

        builder.setNegativeButton(R.string.disrecard,new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        final AlertDialog dia=builder.show();

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
                Intent intent=new Intent(activity, StateMachineEditor.class);
                intent.putExtra("name",text.getText().toString());
                startActivity(intent);
                return true;
            }
        });
    }

}
