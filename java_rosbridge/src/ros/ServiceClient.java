package ros;

import java.util.Map;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;

import com.fasterxml.jackson.databind.JsonNode;

public class ServiceClient {
    private final RosBridge rosBridge;
    private final String serviceName;
    private final String serviceType;

    /**
     * Pending requests waiting for responses.
     * Key = request id
     */
    private final Map<String, CompletableFuture<JsonNode>> pendingRequests = new ConcurrentHashMap<>();

    public ServiceClient(RosBridge rosBridge, String serviceName, String serviceType) {
        this.rosBridge = rosBridge;
        this.serviceName = serviceName;
        this.serviceType = serviceType;
    }

    /**
     * Async service call.
     */
    public CompletableFuture<JsonNode> callAsync(Object request) {
        String requestId = UUID.randomUUID().toString();
        CompletableFuture<JsonNode> future = new CompletableFuture<>();

        pendingRequests.put(requestId, future);
        rosBridge.sendServiceRequest(serviceName, serviceType, request, requestId);

        return future;
    }

    /**
     * Blocking service call.
     */
    public JsonNode call(Object request) throws Exception {
        return callAsync(request).get();
    }

    /**
     * Invoked by RosBridge when a service response arrives.
     */
    public void handleResponse(String requestId, JsonNode response, boolean success) {
        CompletableFuture<JsonNode> future = pendingRequests.remove(requestId);

        if (future == null) {
            return;
        }

        if (success) {
            future.complete(response);
        } else {
            future.completeExceptionally(new RuntimeException("Service call failed: " + serviceName));
        }
    }

    /**
     * Number of pending requests.
     */
    public int getPendingRequestCount() {
        return pendingRequests.size();
    }

    public String getServiceName() {
        return serviceName;
    }

    public String getServiceType() {
        return serviceType;
    }
}