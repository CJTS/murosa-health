import { Pipe, PipeTransform } from '@angular/core';
import { RobotStatus } from '../components/robot/robot';

@Pipe({
  name: 'robotStatus'
})
export class RobotStatusPipe implements PipeTransform {

  transform(value: RobotStatus): string {
    if(value === RobotStatus.CREATED) {
      return "Criado"
    } else if(value === RobotStatus.OCCUPIED) {
      return "Ocupado"
    } else if(value === RobotStatus.RESERVED) {
      return "Reservado"
    }

    return 'Erro'
  }
}
