import { Component, Input } from '@angular/core';
import { IMission, Mission } from '../mission/mission';

@Component({
  selector: 'app-missions',
  imports: [Mission],
  templateUrl: './missions.html',
  styleUrl: './missions.scss'
})
export class Missions {
  @Input('missions') missions: IMission[] = []
}
