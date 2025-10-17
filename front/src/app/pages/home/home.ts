import { Component } from '@angular/core';
import { Map } from '../../components/map/map';
import { MatToolbar } from '@angular/material/toolbar';
import { MatTab, MatTabGroup } from '@angular/material/tabs';
import { MatCard, MatCardModule } from '@angular/material/card';

@Component({
  selector: 'app-home',
  imports: [Map, MatToolbar, MatTabGroup, MatTab, MatCardModule],
  templateUrl: './home.html',
  styleUrl: './home.scss',
})
export class Home {}
