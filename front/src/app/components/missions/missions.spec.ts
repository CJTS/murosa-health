import { ComponentFixture, TestBed } from '@angular/core/testing';

import { Missions } from './missions';

describe('Missions', () => {
  let component: Missions;
  let fixture: ComponentFixture<Missions>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [Missions]
    })
    .compileComponents();

    fixture = TestBed.createComponent(Missions);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
