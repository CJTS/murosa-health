import { ComponentFixture, TestBed } from '@angular/core/testing';

import { Robot } from './robot';

describe('Robot', () => {
  let component: Robot;
  let fixture: ComponentFixture<Robot>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [Robot]
    })
    .compileComponents();

    fixture = TestBed.createComponent(Robot);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
