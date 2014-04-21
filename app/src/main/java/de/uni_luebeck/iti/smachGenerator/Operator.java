package de.uni_luebeck.iti.smachGenerator;

public enum Operator {
    BIGGER(">", "&gt;"),
    BIGGER_EQUAL(">=", "&gt;="),
    EQUAL("==", "=="),
    NOT_EQUAL("!=", "!="),
    SMALLER_EQUAL("<=", "&lt;="),
    SMALLER("<", "&lt;"),
    PLACE_HOLDER("","");

    private String htmlString;
    private String s;

    private Operator(String s, String htmlString) {
        this.s = s;
        this.htmlString = htmlString;
    }

    public String toHTMLString() {
        return htmlString;
    }

    @Override
    public String toString() {
        return s;
    }
}
