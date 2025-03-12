from enum import Enum

class FIPAPerformative(Enum):
    QUERY = 'query'
    REQUEST = 'request'
    AGREE = 'agree'
    INFORM = 'inform'
    FAILURE = 'failure'
    REFUSE = 'refuse'
    SUBSCRIBE = 'subscribe'
