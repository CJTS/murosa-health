import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.logging.Logger;

import com.fasterxml.jackson.databind.JsonNode;

import jason.asSyntax.Literal;
import jason.asSyntax.LiteralImpl;
import jason.asSyntax.Structure;
import jason.asSyntax.Term;
import jason.environment.Environment;
import ros.Publisher;
import ros.RosBridge;
import ros.SubscriptionRequestMsg;
import ros.msgs.std_msgs.PrimitiveMsg;
import ros.tools.MessageUnpacker;

public class RosEnv extends Environment {
	private static final Logger logger = Logger.getLogger("hello_ros." + RosEnv.class.getName());
	RosBridge bridge = new RosBridge();

	/** Called before the MAS execution with the args informed in .mas2j */
	@Override
	public void init(String[] args) {
		super.init(args);
		bridge.connect("ws://localhost:9090", true);
		logger.info("Environment started, connection with ROS established.");

		// Coordinator
		bridge.subscribe(
			SubscriptionRequestMsg.generate("/coordinator/jason/plan").setType("std_msgs/String"),
			(JsonNode data, String stringRep) -> {
				MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
				PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
				FIPAMessage decodedMessage = FIPAMessage.decode(msg.data);
				String regex = "[|]";
				String[] decodedContent = decodedMessage.getContent().split(regex);

				logger.info(msg.data);
				clearPercepts();

				if(decodedMessage.getPerformative().equals("request")) {
					if (decodedContent[0].equals("Create")) {
						Collection<String> collection = new ArrayList<>();
						collection.add("DynamicAgent");
						try {
							getEnvironmentInfraTier().getRuntimeServices().createAgent(
									decodedContent[1],     // agent name
									removeChars(decodedContent[1], 1) + ".asl",       // AgentSpeak source
									null,            // default agent class
									collection,            // default architecture class
									null,            // bbpars
									null,            // settings
									null);           // father
							getEnvironmentInfraTier().getRuntimeServices().startAgent(decodedContent[1]);
						} catch (Exception ex) {
						}
					}
				} else if(decodedMessage.getPerformative().equals("inform")) {
					if (decodedContent[0].equals("Belief")) {
						addPercept(Literal.parseLiteral(decodedContent[1]));
					} else if (decodedContent[0].equals("Action")) {
						String actionsRegex = "[,]";
						String[] actionParts = decodedContent[1].split(actionsRegex);
						Structure act = new Structure(actionParts[0]);

						for (int i = 1; i < actionParts.length; i++) {
							act.addTerm(new LiteralImpl(actionParts[i]));
						}

						Boolean result = executeAction(actionParts[1], act);
						logger.info(result.toString());
					}
				}
			}
		);

		// Robot
		bridge.subscribe(
			SubscriptionRequestMsg.generate("/agent/jason/result").setType("std_msgs/String"),
			(JsonNode data, String stringRep) -> {
				MessageUnpacker<PrimitiveMsg<String>> unpacker = new MessageUnpacker<>(PrimitiveMsg.class);
				PrimitiveMsg<String> msg = unpacker.unpackRosMessage(data);
				FIPAMessage decodedMessage = FIPAMessage.decode(msg.data);
				String regex = "[|]";
				String[] decodedContent = decodedMessage.getContent().split(regex);
				String agentActionRegex = "[,]";
				String[] agents = decodedContent[1].split(agentActionRegex);
				logger.info(msg.data);

				clearPercepts(decodedMessage.getSender());

				if(decodedMessage.getPerformative().equals("inform")) {
					if(decodedContent[0].equals("Success")) {
						addPercept(decodedMessage.getSender(), Literal.parseLiteral("success_" + formatFunction(agents) + ")"));
					}
				}
			}
		);
	}

	@Override
	public boolean executeAction(String agName, Structure action) {
		logger.info(agName + " executing -> " + action + ".");

		if (action.getFunctor().equals("end")) {
			end();
			return true;
		}
		
		Publisher navigation = new Publisher("/jason/agent/action", "std_msgs/String", bridge);
		FIPAMessage message = new FIPAMessage("inform", "jason", agName);
		List<Term> terms = action.getTerms();
		String termsStr = action.getFunctor();

		for (Term term : terms) {
			termsStr +=  "," + term.toString();
		}

		message.setContent(termsStr);
		navigation.publish(new PrimitiveMsg<>(message.encode()));
		// informAgsEnvironmentChanged();
		return true;
	}

	public void end() {
		Publisher navigation = new Publisher("/jason/shutdown_signal", "std_msgs/Bool", bridge);
		navigation.publish(new PrimitiveMsg<>(true));
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

	public static String formatFunction(String[] input) {
		if (input == null || input.length < 2) {
			throw new IllegalArgumentException("O array deve conter pelo menos um nome de função e um parâmetro.");
		}

		String functionName = input[0];
		String parameters = String.join(", ", java.util.Arrays.copyOfRange(input, 1, input.length));

		return functionName + "(" + parameters + ")";
	}
}
