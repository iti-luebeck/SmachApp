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
package de.uni_luebeck.iti.smachapp.model;

import java.io.File;

import de.uni_luebeck.iti.smachapp.model.undo.UndoManager;

/**
 * Created by Morten Mey on 27.04.2014.
 */
public class EditorModel {

    public enum EditorState {
        EDIT_STATES,
        EDIT_TRANSITIONS
    }

    private EditorState currentState = EditorState.EDIT_STATES;

    private StateMachine stateMachine;

    private BeepRobot robot = new BeepRobot();

    private File pythonFile;
    private File saveFile;

    private UndoManager undoManager = new UndoManager();

    public EditorModel(String name) {
        stateMachine = new StateMachine(name);
        updateFileNames();
    }

    public void updateFileNames() {
        File path = XMLSaverLoader.PATH;
        pythonFile = new File(path, stateMachine.getName() + XMLSaverLoader.PYTHON_FILE_ENDING);
        saveFile = new File(path, stateMachine.getName() + XMLSaverLoader.FILE_ENDING);
    }


    public void setCurrentState(EditorState newState) {
        currentState = newState;
    }

    public EditorState getCurrentState() {
        return currentState;
    }

    public StateMachine getStateMachine() {
        return stateMachine;
    }


    public String getNextStateName() {
        int i=stateMachine.getStates().size();

        String res="S"+i;

        while(stateMachine.getState(res)!=null){
            i++;
            res="S"+i;
        }

        return res;
    }

    public String getNextTransitionName() {
        int i=stateMachine.getTransitions().size();

        String res="T"+i;

        while(stateMachine.getTransition(res)!=null){
            i++;
            res="T"+i;
        }

        return res;
    }

    public BeepRobot getRobot() {
        return robot;
    }

    public File getPythonFile() {
        return pythonFile;
    }

    public File getSaveFile() {
        return saveFile;
    }

    public UndoManager getUndoManager() {
        return undoManager;
    }
}
