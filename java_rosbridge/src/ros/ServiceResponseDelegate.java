package ros;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

public interface ServiceResponseDelegate {
	/**
	 * Receives a new published message to a subscribed topic. The JSON data, given as a {@link com.fasterxml.jackson.databind.JsonNode}, has four top-level fields:<p>
	 * op: what kind of operation it was; should always be "publish"<p>
	 * topic: to which topic the message was published<p>
	 * type: the ROS message type of the topic<p>
	 * msg: the provided ros message in JSON format. This the primary field you will work with.<p>
	 * This method also receives the full string representation of the received JSON message from ROSBridge.
	 * @param data the {@link com.fasterxml.jackson.databind.JsonNode} containing the JSON data received.
	 * @param stringRep the string representation of the JSON object.
	 */
	void receive(JsonNode data, String stringRep);


	/**
	 * A class for easy conversion to the legacy java_rosbridge {@link #receive(com.fasterxml.jackson.databind.JsonNode, String)}
	 * message format that presented the JSON data
	 * in a {@link java.util.Map} from {@link java.lang.String} to {@link java.lang.Object} instances
	 * in which the values were ether primitives, {@link java.util.Map} objects themselves, or {@link java.util.List}
	 * objects.
	 */
	public static class LegacyFormat{

		/**
		 * A method for easy conversion to the legacy java_rosbridge {@link #receive(com.fasterxml.jackson.databind.JsonNode, String)}
		 * message format that presented the JSON data
		 * in a {@link java.util.Map} from {@link java.lang.String} to {@link java.lang.Object} instances
		 * in which the values were ether primitives, {@link java.util.Map} objects themselves, or {@link java.util.List}
		 * objects.
		 * @param jsonString the source JSON string message that was received
		 * @return a {@link java.util.Map} data structure of the JSON data.
		 */
		public static Map<String, Object> legacyFormat(String jsonString){

			JsonFactory jsonFactory = new JsonFactory();
			Map<String, Object> messageData = new HashMap<String, Object>();
			try {
				ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
				TypeReference<Map<String, Object>> listTypeRef =
						new TypeReference<Map<String, Object>>() {};
				messageData = objectMapper.readValue(jsonString, listTypeRef);
			} catch (JsonParseException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}

			return messageData;
		}
	}
}
