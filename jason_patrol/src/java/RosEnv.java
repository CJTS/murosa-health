import java.Observation;
import java.util.logging.Logger;

import com.fasterxml.jackson.databind.JsonNode;

import jason.asSyntax.StringTermImpl;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Structure;
import jason.asSyntax.Pred;
import jason.asSyntax.Literal;
import jason.environment.Environment;
import ros.Publisher;
import ros.RosBridge;
import ros.RosListenDelegate;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

public class RosEnv extends Environment {

    private Logger logger = Logger.getLogger("hello_ros."+RosEnv.class.getName());

    RosBridge bridge = new RosBridge();

    /** Called before the MAS execution with the args informed in .mas2j */
    @Override
    public void init(String[] args) {
        super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");

		// Coordinator
		bridge.subscribe(
			SubscriptionRequestMsg.generate("/coordinator/jason/plan")
				.setType("std_msgs/String"),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
					PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
					logger.info(msg.data);
					clearPercepts();
					addPercept(Literal.parseLiteral("start"));
				}
			}
	    );

		// Robot
		bridge.subscribe(
			SubscriptionRequestMsg.generate("/move_base/result")
				.setType("std_msgs/String"),
			new RosListenDelegate() {
				public void receive(JsonNode data, String stringRep) {
					MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(PrimitiveMsg.class);
					PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
					logger.info(msg.data);
					clearPercepts();
					//
					addPercept(Literal.parseLiteral("movebase_result(3)"));
				}
			}
	    );
    }

    @Override
    public boolean executeAction(String agName, Structure action) {
		logger.info("executing: " + action + ": " + agName + ".");

		if (action.getFunctor().equals("move")) {
			move(action.getTerm(0).toString());
		} else if (action.getFunctor().equals("end")) {
			end();
		} else {
			logger.info("executing: "+action+", but not implemented!");
		}
        informAgsEnvironmentChanged();
        return true; // the action was executed with success
    }

	public void move(String waypoint) {
		Publisher navigation = new Publisher("/move_base/move", "std_msgs/String", bridge);
		navigation.publish(new PrimitiveMsg<String>(waypoint));
	}

	public void end() {
		Publisher navigation = new Publisher("/shutdown_signal", "std_msgs/Bool", bridge);
		navigation.publish(new PrimitiveMsg<Boolean>(true));
	}

    /** Called before the end of MAS execution */
    @Override
    public void stop() {
        super.stop();
    }
}
