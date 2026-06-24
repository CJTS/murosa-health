import { Component, Input } from '@angular/core';
import { MatCardModule } from '@angular/material/card';
import { MatCheckboxModule } from '@angular/material/checkbox';
import { RobotStatusPipe } from '../../pipes/robot-status-pipe';
import { JsonPipe } from '@angular/common';
import { Agent } from '../../pages/home/home';

export enum RobotStatus {
  CREATED = 1,
  RESERVED = 2,
  OCCUPIED = 3
}

export interface IMissionRobot {
  robot: string,
  finished: boolean,
  trigger: string,
  plan_version: number,
  current_bdi: string,
  status: RobotStatus,
  ready: boolean
}

@Component({
  selector: 'app-robot',
  imports: [MatCardModule, MatCheckboxModule, RobotStatusPipe, JsonPipe],
  templateUrl: './robot.html',
  styleUrl: './robot.scss'
})
export class Robot {
  @Input('robot') robot!: IMissionRobot;
  @Input('agent') agent!: Agent;
}
