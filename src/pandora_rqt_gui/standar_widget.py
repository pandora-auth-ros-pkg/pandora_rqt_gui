from __future__ import division
import os


from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QTime
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Int16

from .widget_info import WidgetInfo
from .victim_found_server import ValidateVictimAction
from .propability_info import PropabilityInfoWidget



class StandarWidget(QWidget):
    """
    StandarWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin = None ):
       
       
        super(StandarWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'StandarWidget.ui')
        #~ ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'StandarWidget.ui')
        loadUi(ui_file, self)
       
        self._PropabilityInfoWidget = PropabilityInfoWidget(self)
        
        self._validateVictimAction = ValidateVictimAction('victimValidation')
        
        self.score_info= WidgetInfo("chatter", Int16 )
        self.victim_info = WidgetInfo("chatter", Int16 )
        self._victimInfo = [ ]
        
        self.timerStarted = False
        self.victimFound = False
       
        
        
        #Connecting the Buttons
        self.goButton.clicked.connect(self.go_button_clicked)
        self.stopButton.clicked.connect(self.stop_button_clicked)
        self.confirmButton.clicked.connect(self.confirm_victim_clicked)
        self.declineButton.clicked.connect(self.decline_button_clicked)
        
        #Connecting the CheckBoxes
        self.tempCheckBox.stateChanged.connect(self.temp_checked)
        self.co2CheckBox.stateChanged.connect(self.co2_checked)
        self.sonarsCheckBox.stateChanged.connect(self.sonars_checked)
        self.batteryCheckBox.stateChanged.connect(self.battery_checked)
        self.showAllCheckBox.stateChanged.connect(self.show_all)
        #In the Beggining all the checkBoxes are unchecked
        
        self._tempChecked = False
        self._batteryChecked = False
        self._sonarsChecked = False
        self._co2Checked = False
        self._showAllChecked = False 
        
        # Refresh timer
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.score_info.start_monitoring()
        self.victim_info.start_monitoring()
        self._timer_refresh_widget.start(1000)
        
        
    def enableVictimFoundOptions(self):
      
        self.confirmButton.setEnabled(True)
        self.declineButton.setEnabled(True)
        self.victimx.setEnabled(True)
        self.victimy.setEnabled(True)
        self.victimz.setEnabled(True)
        self.sensorID.setEnabled(True)
        self.propability.setEnabled(True)
        self.setVictimInfo()
        self._PropabilityInfoWidget.show()
        self.internalGrid.addWidget(self._PropabilityInfoWidget,1,0)
        self._PropabilityInfoWidget.start()
        
        
    def disableVictimFoundOptions(self):
      
        self.confirmButton.setEnabled(False)
        self.declineButton.setEnabled(False)
        self.victimx.setEnabled(False)
        self.victimy.setEnabled(False)
        self.victimz.setEnabled(False)
        self.sensorID.setEnabled(False)
        self.propability.setEnabled(False)
        self.internalGrid.removeWidget(self._PropabilityInfoWidget)
        self._PropabilityInfoWidget.shutdown()
        self._PropabilityInfoWidget.close()
        
        
    def setVictimInfo(self):
        self.victimx.setText(" Victim X Position " + str(self._victimInfo[0]))
        self.victimy.setText(" Victim Y Position " + str(self._victimInfo[1]))
        self.victimz.setText(" Victim Z Position " + str(self._victimInfo[2]))
        self.sensorID.setText(" Sensor ID " +  "".join(self._victimInfo[4]))
        self.propability.setText(" Probability " + str(self._victimInfo[3]))
       
        
      

    @Slot()
    def refresh_topics(self):

        if self.score_info.last_message is not None:
          self.score.display(self.score_info.last_message.data)
        if self.victim_info.last_message is not None:
          self.victimsFound.display(self.victim_info.last_message.data)
        
        if self.timerStarted:
            self.time2 = self.time2.addSecs(-1)
            self.timer.setTime(self.time2)
        
        self.victimFound  = self._validateVictimAction._victimFound 
        
        if self.victimFound:
          self._victimInfo = self._validateVictimAction._victimInfo
          self.enableVictimFoundOptions()
       
    def go_button_clicked(self):
      
        self.timerStarted = True
        self.time2 = (self.timer.time())
        self.timer.setTime(self.time2)
        
    def stop_button_clicked(self):
      
      self.timerStarted =  False

        
    def decline_button_clicked(self):
      self._validateVictimAction._victimValid = False
      self.disableVictimFoundOptions()
      self._validateVictimAction._operatorResponded = True
      
      
    def confirm_victim_clicked(self):
      self._validateVictimAction._victimValid = True
      self._validateVictimAction._operatorResponded = True
      self.disableVictimFoundOptions()
      
    
    def sonars_checked(self):
      
       self._sonarsChecked = self.sonarsCheckBox.isChecked()
      
    def battery_checked(self):
      
      self._batteryChecked = self.batteryCheckBox.isChecked()
        
    def temp_checked(self):
      
      self._tempChecked = self.tempCheckBox.isChecked()
         
    def co2_checked (self):
      
      self._co2Checked = self.co2CheckBox.isChecked()
      
    def show_all (self):
      self._showAllChecked = self.showAllCheckBox.isChecked()

    def shutdown(self):
      self.victim_info.stop_monitoring()
      self.score_info.stop_monitoring()
      self._timer_refresh_widget.stop()
      self._validateVictimAction.shutdown()
       
  
      
