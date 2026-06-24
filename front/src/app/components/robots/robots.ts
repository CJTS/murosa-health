import { Component, Input } from '@angular/core';
import { Robot, IMissionRobot } from '../robot/robot';
import { Agent } from '../../pages/home/home';

@Component({
  selector: 'app-robots',
  imports: [Robot],
  templateUrl: './robots.html',
  styleUrl: './robots.scss'
})
export class Robots {
  @Input('robots') robots: IMissionRobot[] = []
  @Input('agentsData') agentsData!: Record<string, Agent>;
}
