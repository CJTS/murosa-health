import { Pipe, PipeTransform } from '@angular/core';
import { MissionStatus } from '../components/mission/mission';

@Pipe({
  name: 'missionStatus'
})
export class MissionStatusPipe implements PipeTransform {
  transform(value: MissionStatus): string {
    if(value === MissionStatus.CREATED) {
      return "Criado"
    } else if(value === MissionStatus.WAITING_TEAM) {
      return "Aguardando um time"
    } else if(value === MissionStatus.RUNNING) {
      return "Executando"
    } else if(value === MissionStatus.ERROR) {
      return "Erro encontrado"
    } else if(value === MissionStatus.FINISHED) {
      return "Finalizada"
    } else if(value === MissionStatus.CANCELED) {
      return "Cancelada"
    }

    return 'Erro'
  }
}
