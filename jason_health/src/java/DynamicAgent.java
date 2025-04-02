import java.util.logging.Logger;

import com.fasterxml.jackson.databind.JsonNode;

import jason.architecture.AgArch;
import jason.asSyntax.ASSyntax;
import jason.asSyntax.Plan;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;


public class DynamicAgent extends AgArch {
	RosBridge bridge = new RosBridge();
	private Logger logger = Logger.getLogger("agent." + DynamicAgent.class.getName());

    @Override
    public void init() throws Exception {
        super.init();
        startROSSubscriber();
    }

    private void startROSSubscriber() {
    // Create a ROS subscriber that listens for new plan messages
        bridge.connect("ws://localhost:9090", true);

        bridge.subscribe(
        SubscriptionRequestMsg.generate("/coordinator/agent/plan")
                .setType("std_msgs/String"),
        new RosListenDelegate() {
            public void receive(JsonNode data, String stringRep) {
                MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
                    PrimitiveMsg.class);
                PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
                logger.info(msg.data);
                FIPAMessage decodedMessage = FIPAMessage.decode(msg.data);
                String regex = "[|]";
                String[] decodedContent = decodedMessage.getContent().split(regex);

                if(decodedMessage.getPerformative().equals("request")) {
                    if(decodedContent[0].equals("Plan")) {
                        addPlanDynamically(decodedContent[1]);
                    }
                }
            }
        });
    };

    private void addPlanDynamically(String planStr) {
        try {
            Plan newPlan = ASSyntax.parsePlan(receivePlanFromROS());  // Parse string into Jason plan
            getTS().getAg().getPL().add(newPlan);        // Add plan to agent's plan library
            System.out.println("New plan added: " + newPlan);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private String receivePlanFromROS() {
        // Implement ROS subscriber logic here
        return "+!goal : true <- .print(\"Executing new plan\").";  // Example static plan
    }
}