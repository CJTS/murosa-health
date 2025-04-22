import com.google.gson.Gson;

class FIPAMessage {
    private final String performative;
    private final String sender;
    private final String receiver;
    private String content;

    public String getPerformative() {
        return performative;
    }

    public String getSender() {
        return sender;
    }

    public String getReceiver() {
        return receiver;
    }

    public void setContent(String content) {
        this.content = content;
    }

    public String getContent() {
        return content;
    }

    public FIPAMessage(String performative, String sender, String receiver) {
        this.performative = performative;
        this.sender = sender;
        this.receiver = receiver;
    }

    public String encode() {
        Gson gson = new Gson();
        return gson.toJson(this);
    }

    public static FIPAMessage decode(String messageStr) {
        Gson gson = new Gson();
        return gson.fromJson(messageStr, FIPAMessage.class);
    }

    @Override
    public String toString() {
        return "FIPAMessage{" +
                "performative='" + performative + '\'' +
                ", sender='" + sender + '\'' +
                ", receiver='" + receiver + '\'' +
                ", content='" + content + '\'' +
                '}';
    }
}