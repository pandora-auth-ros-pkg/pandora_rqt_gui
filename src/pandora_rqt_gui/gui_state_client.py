import rospy

from state_manager.state_client import StateClient



class GuiStateClient(StateClient):
  


    def __init__(self):
      
       super(GuiStateClient,self).__init__()

       self._name = "Gui State Client"
       self._state = 1
       self._int_to_state_dict = {0: "MODE_OFF",
                                  1: "MODE_START_AUTONOMOUS",
                                  2: "MODE_EXPLORATION",
                                  3: "MODE_IDENTIFICATION",
                                  4: "MODE_ARM_APPROACH",
                                  5: "MODE_DF_HOLD",
                                  6: "MODE_SEMI_AUTONOMOUS", 
                                  7: "MODE_TELEOPERATED_LOCOMOTION",
                                  8: "MODE_ARM_TELEOPERATION",
                                  9: "MODE_ARM_TELEOPERATION_TUCK",
                                  10: "MODE_TERMINATING"
                                  }


    def start_transition(self, state):
        rospy.loginfo("[%s] Starting Transition to state %i", self._name, state)
        self._state = state
        self.transition_complete(state)
    
    def getState(self):
        return self._int_to_state_dict[self._state]
        
        
