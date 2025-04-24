import java.util.logging.Logger;

import com.fasterxml.jackson.databind.JsonNode;

import jason.JasonException;
import jason.RevisionFailedException;
import jason.architecture.AgArch;
import jason.asSyntax.ASSyntax;
import jason.asSyntax.Literal;
import jason.asSyntax.Plan;
import jason.asSyntax.parser.ParseException;
import ros.Publisher;
import ros.RosBridge;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

public class DynamicAgent extends AgArch {
	private static final Logger logger = Logger.getLogger("agent." + DynamicAgent.class.getName());
	RosBridge bridge = new RosBridge();

    @Override
    public void init() throws Exception {
        super.init();
        startROSSubscriber();
    }

    private void startROSSubscriber() {
        // Create a ROS subscriber that listens for new plan messages
        bridge.connect("ws://localhost:9090", true);

        bridge.subscribe(
            SubscriptionRequestMsg.generate("/coordinator/agent/plan").setType("std_msgs/String"),
            (JsonNode data, String stringRep) -> {
                MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
                PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                FIPAMessage decodedMessage = FIPAMessage.decode(msg.data);

                if(decodedMessage.getReceiver().equals(getAgName())) {
                    String regex = "[|]";
                    String[] decodedContent = decodedMessage.getContent().split(regex);

                    if(decodedMessage.getPerformative().equals("request")) {
                        if(decodedContent[0].equals("Start")) {
                            try {
                                getTS().getAg().addBel(Literal.parseLiteral("start(" + decodedContent[1] + ")"));
                            } catch (RevisionFailedException ex) {
                                System.err.println("Error: " + ex.getMessage());
                            }
                        }
                    } else if(decodedMessage.getPerformative().equals("inform")) {
                        if(decodedContent[0].equals("Plan")) {
                            addPlanDynamically(decodedContent[1]);
                        } else if (decodedContent[0].equals("Belief")) {
                            try {
                                getTS().getAg().addBel(Literal.parseLiteral(decodedContent[1]));
                            } catch (RevisionFailedException ex) { 
                                System.err.println("Error: " + ex.getMessage());
                            }
                        }
                    }
                }
            }
        );

		FIPAMessage message = new FIPAMessage("inform", getAgName(), "coordinator");
		message.setContent("Ready");
        Publisher navigation = new Publisher("/agent/coordinator/action", "std_msgs/String", bridge);
		navigation.publish(new PrimitiveMsg<>(message.encode()));
    };

    private void addPlanDynamically(String planStr) {
        try {
            getTS().getAg().getPL().clear();
            String regex = "[/]";
            String[] plans = planStr.split(regex);

            for (String plan : plans) {
                Plan newPlan = ASSyntax.parsePlan(plan);
                getTS().getAg().getPL().add(newPlan);
            }
        } catch (JasonException | ParseException e) {
            System.err.println("Error: " + e.getMessage());
        }
    }

    @Override
    public void stop() {
        bridge.closeConnection();
    }
}