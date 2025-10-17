import { Component, OnInit } from '@angular/core';
import { Map } from '../../components/map/map';
import { MatToolbar } from '@angular/material/toolbar';
import { MatTab, MatTabGroup } from '@angular/material/tabs';
import { MatCard, MatCardModule } from '@angular/material/card';
import { Websocket } from '../../services/websocket/websocket';
import { FIPAMessage } from '../../models/FIPAMessage';

@Component({
  selector: 'app-home',
  imports: [Map, MatToolbar, MatTabGroup, MatTab, MatCardModule],
  templateUrl: './home.html',
  styleUrl: './home.scss',
})
export class Home implements OnInit{
  state: any = {}

  constructor(private websocket: Websocket) {}

  get agents(): string[] {
    if(this.state && this.state?.pos) {
      return Object.keys(this.state?.pos)
    } 

    return []
  }

  ngOnInit(): void {
    this.websocket.connect("ws://localhost:9090");
    this.websocket.getMessages().subscribe(response => {
      const message = FIPAMessage.decode(response.msg.data);
      this.state = JSON.parse(message.content);
    });
    this.websocket.sendMessage({
      op: 'subscribe',
      topic: '/env/front/state',
      type: 'std_msgs/String',
    });
  }
}
