class MissionManager():
    def __init__(self):
    
        if(team == None):
            if room == 'icu':
                self.get_logger().info("ERROR|icu_room")
                self.stop_low_priority_mission()
                time.sleep(1)
                team = self.get_team(room)
                if(team == None):
                    self.get_logger().info("Cant make team")
                    return
                start_context = self.get_start_context(team, room)
                self.create_mission(team, start_context, room)
            self.create_empty_mission(room)
            return
