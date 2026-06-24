import { Component, Input } from '@angular/core';
import { MatCardModule } from '@angular/material/card';
import { MatCheckboxModule } from '@angular/material/checkbox';
import { RobotStatusPipe } from '../../pipes/robot-status-pipe';
import { IMissionRobot } from '../robot/robot';
import { MissionStatusPipe } from '../../pipes/mission-status-pipe';
import { JsonPipe } from '@angular/common';

export enum MissionStatus {
  CREATED = 1,
  WAITING_TEAM = 2,
  RUNNING = 3,
  ERROR = 5,
  FINISHED = 6,
  CANCELED = 7
}

export interface IMission {
  team: IMissionRobot,
  status: MissionStatus,
  context: any,
  priority: number,
  mission_context: string,
  variables: string[],
  plan: string[],
  state: any,
  error: string,
  requester: string,
  params: string[]
}

@Component({
  selector: 'app-mission',
  imports: [
    MatCardModule,
    MatCheckboxModule,
    MissionStatusPipe,
    JsonPipe
  ],
  templateUrl: './mission.html',
  styleUrl: './mission.scss'
})
export class Mission {
  @Input('mission') mission!: IMission;
}
