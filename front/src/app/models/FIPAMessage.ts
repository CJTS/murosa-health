export class FIPAMessage {
  performative: string;
  sender: string;
  receiver: string;
  content: any;

  constructor(performative: string, sender: string, receiver: string, content: any) {
    this.performative = performative;
    this.sender = sender;
    this.receiver = receiver;
    this.content = content;
  }

  static decode(messageStr: string): FIPAMessage {
    const data = JSON.parse(messageStr);
    return new FIPAMessage(data.performative, data.sender, data.receiver, data.content);
  }
}