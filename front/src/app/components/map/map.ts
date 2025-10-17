import { AfterViewInit, Component, effect, ElementRef, input, ViewChild } from '@angular/core';

@Component({
  selector: 'app-map',
  imports: [],
  templateUrl: './map.html',
  styleUrl: './map.scss',
})
export class Map implements AfterViewInit {
  @ViewChild('hospitalCanvas', { static: false }) hospitalCanvas!: ElementRef<HTMLCanvasElement>;
  private ctx!: CanvasRenderingContext2D;
  agents: string[] = [];

  state = input<any>();

  rooms = [
    { x: 10, y: 10, width: 50, height: 50, label: 'room1' },
    { x: 60, y: 10, width: 50, height: 50, label: 'room2' },
    { x: 110, y: 10, width: 50, height: 50, label: 'room3' },
    { x: 160, y: 10, width: 50, height: 50, label: 'lab' },
    { x: 210, y: 10, width: 50, height: 50, label: 'ds' },
    { x: 260, y: 10, width: 140, height: 140, label: 'icu' },

    { x: 10, y: 100, width: 50, height: 50, label: 'room4' },
    { x: 60, y: 100, width: 50, height: 50, label: 'room5' },
    { x: 110, y: 100, width: 50, height: 50, label: 'room6' },
    { x: 160, y: 100, width: 50, height: 50, label: 'nr' },
  ];

  intersections = [
    { x: 35, y: 80 },
    { x: 85, y: 80 },
    { x: 135, y: 80 },
    { x: 185, y: 80 },
    { x: 235, y: 80 },
  ];

  constructor() {
    effect(() => {
      if (this.state() && this.state()?.pos) {
        this.agents = Object.keys(this.state()?.pos);
      }

      if (this.ctx) {
        this.ctx.reset();
        this.drawLayout();
        this.drawAgents();
      }
    });
  }

  ngAfterViewInit(): void {
    const canvas = this.hospitalCanvas.nativeElement;
    this.ctx = canvas.getContext('2d') as CanvasRenderingContext2D;

    this.drawLayout();
  }

  drawLayout(): void {
    const ctx = this.ctx;

    // === CONFIGURAÇÕES ===
    ctx.lineWidth = 3;
    ctx.strokeStyle = '#000';

    // === QUARTOS PACIENTES (R) ===
    this.rooms.forEach((room) => {
      if (this.state()?.samples && this.state()?.samples[room.label]) {
        ctx.fillStyle = 'blue';
      } else {
        ctx.fillStyle = '#000';
      }

      ctx.strokeRect(room.x, room.y, room.width, room.height);
      ctx.font = '14px Arial';
      ctx.fillText(room.label, room.x + 3, room.y + 15);
    });

    ctx.fillStyle = '#000';

    // === Caminhos (Path Segments) ===
    ctx.setLineDash([4, 2]);
    ctx.beginPath();
    ctx.moveTo(0, 80);
    ctx.lineTo(260, 80);
    ctx.moveTo(35, 60);
    ctx.lineTo(35, 100);
    ctx.moveTo(85, 60);
    ctx.lineTo(85, 100);
    ctx.moveTo(135, 60);
    ctx.lineTo(135, 100);
    ctx.moveTo(185, 60);
    ctx.lineTo(185, 100);
    ctx.moveTo(235, 60);
    ctx.lineTo(235, 180);
    ctx.stroke();
    ctx.setLineDash([]);

    // === Intersections (losangos) ===
    this.intersections.forEach((i) => {
      ctx.beginPath();
      ctx.moveTo(i.x, i.y - 8);
      ctx.lineTo(i.x + 8, i.y);
      ctx.lineTo(i.x, i.y + 8);
      ctx.lineTo(i.x - 8, i.y);
      ctx.closePath();
      ctx.stroke();
    });

    // === Legenda ===
    ctx.fillStyle = '#000';
    ctx.strokeRect(410, 10, 170, 125);
    ctx.font = '14px Arial';
    ctx.fillText('Legend:', 413, 25);

    ctx.beginPath();
    ctx.moveTo(425, 40 - 8);
    ctx.lineTo(425 + 8, 40);
    ctx.lineTo(425, 40 + 8);
    ctx.lineTo(425 - 8, 40);
    ctx.closePath();
    ctx.stroke();
    ctx.fillText('Intersections', 440, 45);

    ctx.setLineDash([6, 4]);
    ctx.beginPath();
    ctx.moveTo(417, 60);
    ctx.lineTo(433, 60);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillText('Path Segment', 440, 65);

    ctx.fillText('icu = Intensive Care Unit', 417, 85);
    ctx.fillText('nr = Nurses Room', 417, 105);
    ctx.fillText('ds = Docking Station', 417, 125);
  }

  drawAgents(): void {
    const ctx = this.ctx;

    this.agents.forEach((agentName) => {
      const agent = this.state()?.pos[agentName];

      if (agentName.includes('arm')) {
        ctx.fillStyle = 'red';
      } else if (agentName.includes('collector')) {
        ctx.fillStyle = 'blue';
      } else if (agentName.includes('spot')) {
        ctx.fillStyle = 'yellow';
      } else if (agentName.includes('uvd')) {
        ctx.fillStyle = 'green';
      } else if (agentName.includes('nurse')) {
        ctx.fillStyle = 'black';
      }

      ctx.beginPath();
      ctx.arc(agent[0], agent[1], 5, 0, Math.PI * 2);
      ctx.fill();
    });

    ctx.fillStyle = '#000';
  }
}
