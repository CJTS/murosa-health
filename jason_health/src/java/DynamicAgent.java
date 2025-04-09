import java.util.logging.Logger;

import com.fasterxml.jackson.databind.JsonNode;

import jason.JasonException;
import jason.architecture.AgArch;
import jason.asSyntax.ASSyntax;
import jason.asSyntax.Plan;
import jason.asSyntax.parser.ParseException;
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
                    logger.info(msg.data);
                    String regex = "[|]";
                    String[] decodedContent = decodedMessage.getContent().split(regex);

                    if(decodedMessage.getPerformative().equals("inform")) {
                        if(decodedContent[0].equals("Plan")) {
                            addPlanDynamically(decodedContent[1]);
                        }
                    }
                }
            }
        );
    };

    private void addPlanDynamically(String planStr) {
        try {
            System.out.println(getAgName() + " " + planStr);
            Plan newPlan = ASSyntax.parsePlan(planStr);
            System.out.println(getAgName() + " " + newPlan.getTrigger());
            System.out.println(getAgName() + " " + getTS().getAg().getPL().hasCandidatePlan(newPlan.getTrigger()));
            if(!getTS().getAg().getPL().hasCandidatePlan(newPlan.getTrigger())) {
                getTS().getAg().getPL().add(newPlan);
                System.out.println("New plan added: " + newPlan);
            }
        } catch (JasonException | ParseException e) {
        }
    }
}