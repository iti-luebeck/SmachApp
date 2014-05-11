package de.uni_luebeck.iti.smachapp.model;

/**
 * Created by Morten Mey on 27.04.2014.
 */
public class EditorModel {

    public enum EditorState{
        EDIT_STATES,
        EDIT_TRANSITIONS
    }

    private EditorState currentState=EditorState.EDIT_STATES;

    private StateMachine stateMachine=new StateMachine();

    private int stateNameCounter=0;
    private int transitionNameCounter=0;

    public void setCurrentState(EditorState newState){
        currentState=newState;
    }

    public EditorState getCurrentState(){
        return currentState;
    }

    public StateMachine getStateMachine(){
        return stateMachine;
    }

    public void setStateNameCounter(int i){
        stateNameCounter=i;
    }

    public String getNextStateName(){
        String res="S"+stateNameCounter;
        stateNameCounter++;
        return res;
    }

    public void setTransitionNameCounter(int i){
        transitionNameCounter=i;
    }

    public String getNextTransitionName(){
        String res="T"+transitionNameCounter;
        transitionNameCounter++;
        return res;
    }
}
