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
package de.uni_luebeck.iti.smachapp.app;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.text.InputType;
import android.view.KeyEvent;
import android.view.View;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;

import de.uni_luebeck.iti.smachapp.model.XMLSaverLoader;

public class MainActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        XMLSaverLoader.PATH.mkdirs();
    }

    public void newFSM(View view) {

        AlertDialog.Builder builder = new AlertDialog.Builder(this);

        builder.setTitle(R.string.enterMachineName);
        final EditText text = new EditText(this);
        text.setInputType(InputType.TYPE_CLASS_TEXT);
        text.setImeOptions(EditorInfo.IME_ACTION_NEXT);

        String baseName = getString(R.string.newMachine);
        String name = baseName;

        for (int i = 1; XMLSaverLoader.doesFileExist(name); i++) {
            name = baseName + i;
        }

        text.setText(name);
        text.selectAll();
        builder.setView(text);

        final Activity activity = this;
        final Intent intent = new Intent(activity, StateMachineEditor.class);
        intent.putExtra("action", "new");

        builder.setPositiveButton(R.string.next, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                text.setSelection(0);
                handlePositive(text.getText().toString(), intent);
            }
        });

        builder.setNegativeButton(R.string.disrecard, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        final AlertDialog dia = builder.show();

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
                handlePositive(text.getText().toString(), intent);
                return true;
            }
        });
    }

    private void handlePositive(String name, final Intent intent) {

        final String newName=name.trim();

        if(newName.isEmpty()){
            Toast toast=Toast.makeText(this, R.string.empty_name,Toast.LENGTH_LONG);
            toast.show();
            return;
        }

        Context activity = this;
        if (XMLSaverLoader.doesFileExist(newName)) {
            AlertDialog.Builder build = new AlertDialog.Builder(activity);
            build.setTitle(R.string.file_already_exists);
            build.setMessage(R.string.file_already_exists_message);
            build.setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {
                    intent.putExtra("name", newName);
                    startActivity(intent);
                }
            });
            build.setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialogInterface, int i) {

                }
            });
            build.show();
        } else {
            intent.putExtra("name", newName);
            startActivity(intent);
        }
    }

    public void load(View view) {

        final File[] files = XMLSaverLoader.getAllSaves();

        if (files.length == 0) {
            Toast toast = Toast.makeText(this, R.string.no_files_to_load, Toast.LENGTH_LONG);
            toast.show();
            return;
        }

        String[] names = new String[files.length];

        for (int i = 0; i < files.length; i++) {
            names[i] = files[i].getName().substring(0, files[i].getName().lastIndexOf("."));
        }

        final Intent intent = new Intent(this, StateMachineEditor.class);
        intent.putExtra("action", "load");

        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(R.string.pick_a_file)
                .setItems(names, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {
                        intent.setData(Uri.fromFile(files[which]));
                        startActivity(intent);
                    }
                });

        builder.show();
    }

    public void delete(View view) {
        final File[] files = XMLSaverLoader.getAllSaves();

        if (files.length == 0) {
            Toast toast = Toast.makeText(this, R.string.no_files_to_load, Toast.LENGTH_LONG);
            toast.show();
            return;
        }

        String[] names = new String[files.length];

        for (int i = 0; i < files.length; i++) {
            names[i] = files[i].getName().substring(0, files[i].getName().lastIndexOf("."));
        }

        final boolean[] selected = new boolean[names.length];

        AlertDialog.Builder builder = new AlertDialog.Builder(this);
        builder.setTitle(R.string.delete_machine_title);
        builder.setMultiChoiceItems(names, null, new DialogInterface.OnMultiChoiceClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i, boolean b) {
                selected[i] = b;
            }
        });

        builder.setPositiveButton(R.string.delete, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int blah) {
                for (int i = 0; i < files.length; i++) {
                    if (selected[i]) {
                        files[i].delete();
                    }
                }
            }
        });

        builder.setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {

            }
        });

        builder.show();
    }

    public void gotoSettings(View view) {
        Intent intent = new Intent(this, SettingsActivity.class);
        startActivity(intent);
    }

}
