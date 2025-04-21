
import json

def action_string_to_tuple(actionString):
    return tuple(actionString.split(','))

def action_tuple_to_string(actionTuple):
    return ','.join(actionTuple)

class FIPAMessage:
    def __init__(self, performative, sender, receiver, content):
        self.performative = performative
        self.sender = sender
        self.receiver = receiver
        self.content = content

    def encode(self):
        return json.dumps({
            'performative': self.performative,
            'sender': self.sender,
            'receiver': self.receiver,
            'content': self.content
        })

    @staticmethod
    def decode(message_str):
        data = json.loads(message_str)
        return FIPAMessage(data['performative'], data['sender'], data['receiver'], data['content'])
