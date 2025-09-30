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
						String createRegex = "[,]";
						String[] decodedCreateContent = decodedContent[1].split(createRegex);

						try {
							getEnvironmentInfraTier().getRuntimeServices().createAgent(
									decodedCreateContent[0],     // agent name
									decodedCreateContent[1] + ".asl",       // AgentSpeak source
									null,            // default agent class
									collection,            // default architecture class
									null,            // bbpars
									null,            // settings
									null);           // father
							getEnvironmentInfraTier().getRuntimeServices().startAgent(decodedCreateContent[0]);
						} catch (Exception ex) {
						}
					} else if (decodedContent[0].equals("End")) {
						getEnvironmentInfraTier().getRuntimeServices().killAgent(decodedContent[1], "", 0);
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
				String agentActionRegex = "[,]";

				String[] decodedContent = decodedMessage.getContent().split(regex);
				String[] agents = new String[0];

				if(decodedContent.length >= 2) {
					agents = decodedContent[1].split(agentActionRegex);
				}
				
				logger.info(msg.data);

				clearPercepts(decodedMessage.getSender());

				if(decodedMessage.getPerformative().equals("inform")) {
					if(decodedContent[0].equals("Success")) {
						addPercept(decodedMessage.getSender(), Literal.parseLiteral("success_" + formatFunction(agents) + ")"));
					} else if(decodedContent[0].equals("Failure")) {
						addPercept(decodedMessage.getSender(), Literal.parseLiteral("failure_" + formatFunction(agents) + ")"));
					} else if(decodedContent[0].equals("BatteryFailure")) {
						addPercept(decodedMessage.getSender(), Literal.parseLiteral("low_battery_failure(" + formatFunction(agents) + "))"));
					}
				} else if (decodedMessage.getPerformative().equals("request")) {
					if (decodedContent[0].equals("End")) {
						getEnvironmentInfraTier().getRuntimeServices().killAgent(decodedContent[1], "", 0);
					}
				}
			}
		);
	}

	@Override
	public boolean executeAction(String agName, Structure action) {
		logger.info(agName + " executing -> " + action + ".");
		Publisher navigation = new Publisher("/jason/agent/action", "std_msgs/String", bridge);
		FIPAMessage message = new FIPAMessage("inform", "jason", agName);

		if (action.getFunctor().equals("end")) {
			message.setContent("Mission Completed");
		} else {
			List<Term> terms = action.getTerms();
			String termsStr = action.getFunctor();

			if(terms != null && !terms.isEmpty()) {
				for (Term term : terms) {
					termsStr +=  "," + term.toString();
				}
			}

			message.setContent(termsStr);
		}

		navigation.publish(new PrimitiveMsg<>(message.encode()));
		return true;
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
		if (input == null || input.length < 1) {
			throw new IllegalArgumentException("O array deve conter pelo menos um nome de função e um parâmetro.");
		}

		if(input.length == 1) {
			return input[0];
		}

		String functionName = input[0];
		String parameters = String.join(", ", java.util.Arrays.copyOfRange(input, 1, input.length));

		return functionName + "(" + parameters + ")";
	}
}
