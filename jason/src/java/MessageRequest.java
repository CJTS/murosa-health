package src.java;

class MessageRequest {
    String content;

    MessageRequest () { }

    MessageRequest (String content) {
        this.content = content;
    }

    public String getContent() {
        return content;
    }

    public void setContent(String content) {
        this.content = content;
    }
}