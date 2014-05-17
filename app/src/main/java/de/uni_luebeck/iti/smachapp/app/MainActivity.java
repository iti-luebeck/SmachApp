package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.text.InputType;
import android.view.View;
import android.widget.EditText;

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
        text.setText(R.string.newMachine);
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

        builder.show();
    }

}
