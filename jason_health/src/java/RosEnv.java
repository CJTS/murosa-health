import java.util.logging.Level;
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

	private Logger logger = Logger.getLogger("hello_ros." + RosEnv.class.getName());

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
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
								PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						logger.info(msg.data);
						FIPAMessage decodedMessage = FIPAMessage.decode(msg.data);
						String regex = "[|]";
						String[] decodedContent = decodedMessage.getContent().split(regex);

						if(decodedMessage.getPerformative().equals("request")) {
							if(decodedContent[0].equals("Start")) {
								String agentRegex = "[,]";
								String[] agents = decodedContent[1].split(agentRegex);
								for (String agent: agents) {
									logger.info(agent.toLowerCase());
									clearPercepts(agent.toLowerCase());
									addPercept(agent.toLowerCase(), Literal.parseLiteral("start"));
								}
							} else if (decodedContent[0].equals("Create")) {
								try {
									getEnvironmentInfraTier().getRuntimeServices().createAgent(
											decodedContent[1].toLowerCase(),     // agent name
											removeChars(decodedContent[1].toLowerCase(), 1) + ".asl",       // AgentSpeak source
											null,            // default agent class
											null,            // default architecture class
											null,            // default architecture class
											null,            // default architecture class
											null);           // default settings
											getEnvironmentInfraTier().getRuntimeServices().startAgent(decodedContent[1].toLowerCase());
								} catch (Exception ex) {
								}
							}
						}
					}
				});

		// Robot
		bridge.subscribe(
				SubscriptionRequestMsg.generate("/agent/jason/result")
						.setType("std_msgs/String"),
				new RosListenDelegate() {
					public void receive(JsonNode data, String stringRep) {
						MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<PrimitiveMsg<String>>(
								PrimitiveMsg.class);
						PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
						logger.info(msg.data);
						clearPercepts();
						addPercept(Literal.parseLiteral("movebase_result(3)"));
					}
				});
	}

	@Override
	public boolean executeAction(String agName, Structure action) {
		logger.info("executing: " + action + ": " + agName + ".");
		Publisher navigation = new Publisher("/jason/agent/action", "std_msgs/String", bridge);
		FIPAMessage message = new FIPAMessage("inform", "jason", agName);
		String agentAction = "";

		switch (action.getFunctor()) {
			case "a_navto":
			case "a_open_door":
			case "a_approach_nurse":
			case "a_authenticate_nurse":
			case "a_deposit":
			case "a_approach_arm":
			case "a_pick_up_sample":
				message.setContent(action.getFunctor() + "," + action.getTerm(0).toString() + "," + action.getTerm(1).toString());
				break;
			case "a_open_drawer":
			case "a_close_drawer":
				message.setContent(action.getFunctor() + "," + action.getTerm(0).toString());
				break;
			case "end":
				end();
				break;
			default:
				logger.log(Level.INFO, "executing: {0}, but not implemented!", action);
				break;
		}

		agentAction = message.encode();
		navigation.publish(new PrimitiveMsg<String>(agentAction));
		informAgsEnvironmentChanged();
		return true; // the action was executed with success
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

	public static String removeChars(String str, int numberOfCharactersToRemove) {
		if(str != null && !str.trim().isEmpty()) {
			return str.substring(0, str.length() - numberOfCharactersToRemove);
		}
		return "";
	}
}
