import { Component, OnInit } from '@angular/core';
import { Map } from '../../components/map/map';
import { MatToolbar } from '@angular/material/toolbar';
import { MatTab, MatTabGroup } from '@angular/material/tabs';
import { MatCardModule } from '@angular/material/card';
import { Websocket } from '../../services/websocket/websocket';
import { FIPAMessage } from '../../models/FIPAMessage';
import { MatInputModule } from '@angular/material/input';
import { FormControl, FormsModule, ReactiveFormsModule } from '@angular/forms';
import { map, Observable, of, startWith } from 'rxjs';
import { AsyncPipe } from '@angular/common';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatSelectModule } from '@angular/material/select';
import { Robots } from '../../components/robots/robots';
import { Missions } from '../../components/missions/missions';
import { MatIconModule } from '@angular/material/icon';
import { MatButtonModule } from '@angular/material/button';

export type Agent = {
  name: string,
  finished_actions: string[],
  actions: string,
  plan: string,
  wating_response: string,
  wating: string,
  should_use_bdi: string,
  with_plan: string,
  moving: string,
  current: string,
  goal: string,
  path: string,
  vx: string,
  vy: string,
  mission: string,
  local: string,
  _from_local_replan: string,
  _local_replan_enabled: string,
}

@Component({
  selector: 'app-home',
  imports: [
    FormsModule,
    ReactiveFormsModule,
    Map,
    Robots,
    Missions,
    MatToolbar,
    MatTabGroup,
    MatTab,
    MatCardModule,
    MatSelectModule,
    MatFormFieldModule,
    MatInputModule,
    MatButtonModule,
    AsyncPipe
  ],
  templateUrl: './home.html',
  styleUrl: './home.scss',
})
export class Home implements OnInit {
  state: any = {}
  agentsData: Record<string, Agent> = {}
  agentFilter = new FormControl('');
  filteredOptions: Observable<string[]> = of([]);
  selectedAgent: string = '';

  constructor(private websocket: Websocket) {
    setTimeout(() => {
      this.filteredOptions = this.agentFilter.valueChanges.pipe(
        startWith(''),
        map(value => this._filter(value || '')),
      );
    }, 500)
  }

  get agents(): string[] {
    if(this.state && this.state?.pos) {
      return Object.keys(this.state?.pos)
    }

    return []
  }

  get coordinator(): any {
    if(this.agentsData && this.agentsData['coordinator']) {
      return this.agentsData['coordinator']
    }

    return null
  }

  ngOnInit(): void {
    this.websocket.connect("ws://localhost:9090");
    this.websocket.getMessages().subscribe(response => {
      const message = FIPAMessage.decode(response.msg.data);
      if(message.sender === "Env") {
        this.state = JSON.parse(message.content);
      } else {
        this.agentsData[message.sender] = JSON.parse(message.content);
      }
    });
    this.websocket.sendMessage({
      op: 'subscribe',
      topic: '/env/front/state',
      type: 'std_msgs/String',
    });
    this.websocket.sendMessage({
      op: 'subscribe',
      topic: '/agent/front/state',
      type: 'std_msgs/String',
    });
    this.websocket.sendMessage({
      op: 'subscribe',
      topic: '/coordinator/front/state',
      type: 'std_msgs/String',
    });
    this.websocket.sendMessage({
      op: 'advertise',
      topic: '/front/start',
      type: 'std_msgs/String',
    });
  }

  private _filter(value: string): string[] {
    const filterValue = value.toLowerCase();
    return this.agents.filter(agent => agent.toLowerCase().includes(filterValue));
  }

  startMission() {
    console.log(1)
    this.websocket.sendMessage({
      op: 'publish',
      topic: '/front/start',
      type: 'std_msgs/String',
      msg: { "data": "start" }
    });
  }
}
