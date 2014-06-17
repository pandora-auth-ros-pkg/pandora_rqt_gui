import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QTime
from python_qt_binding.QtGui import QWidget, QPixmap
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Int16

from .widget_info import WidgetInfo
from .victim_found_server import ValidateVictimActionServer
from .probability_info import ProbabilityInfoWidget
from .console import Console





class StandarWidget(QWidget):
    """
    StandarWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(StandarWidget, self).__init__()
        
        
        
        #Load Ui
        self._rp = rospkg.RosPack()
        ui_file = os.path.join(self._rp.get_path('pandora_rqt_gui'), 'resources', 'StandarWidget.ui')
        loadUi(ui_file, self)

        self._Console = Console()
        self._ConsoleWidget = self._Console._widget
        self._ProbabilityInfoWidget = ProbabilityInfoWidget(self)

        #Add Console in the 2,1 position of InternalGrid
        self.internalGrid.addWidget(self._ConsoleWidget, 2, 1)

        #use full ABSOLUTE path to the image, not relative
        self.image.setPixmap(QPixmap(os.path.join(self._rp.get_path('pandora_rqt_gui'), 'images', 'pandora_logo.jpeg')))

        # the ValidateVictimActionServer is used called when a victim is found
        self.ValidateVictimActionServer = ValidateVictimActionServer('victimValidation')

        #Subscribe the score the victims_number and Info
        self.score_info = WidgetInfo("chatter", Int16)
        self.victims_number = WidgetInfo("chatter", Int16)
        self._victimInfo = []

        self.timerStarted = False

        #Connecting the Buttons
        self.goButton.clicked.connect(self.go_button_clicked)
        self.stopButton.clicked.connect(self.stop_button_clicked)
        self.confirmButton.clicked.connect(self.confirm_victim_clicked)
        self.declineButton.clicked.connect(self.decline_button_clicked)
        self.left_panel_button.clicked.connect(self.left_panel_button_clicked)
        self.search_and_rescue_button.clicked.connect(self.search_and_rescue_button_clicked)
        self.mapping_mission_button.clicked.connect(self.mapping_mission_button_clicked)
        self.teleop_state_button.clicked.connect(self.teleop_state_button_clicked)
        self.semi_autonomous_state.clicked.connect(self.semi_autonomous_state_clicked)
        
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
        
        #The left panel is visible
        self._left_panel = True

        # Refresh timer
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):

        self.score_info.start_monitoring()
        self.victims_number.start_monitoring()
        self._timer_refresh_widget.start(1000)

    def enableVictimFoundOptions(self):

        self.confirmButton.setEnabled(True)
        self.declineButton.setEnabled(True)
        self.victimx.setEnabled(True)
        self.victimy.setEnabled(True)
        self.victimz.setEnabled(True)
        self.sensorID.setEnabled(True)
        self.probability.setEnabled(True)
        self.setVictimInfo()
        self._ProbabilityInfoWidget.show()
        self.internalGrid.addWidget(self._ProbabilityInfoWidget, 1, 0)
        self._ProbabilityInfoWidget.start()

    def disableVictimFoundOptions(self):

        self.confirmButton.setEnabled(False)
        self.declineButton.setEnabled(False)
        self.victimx.setEnabled(False)
        self.victimy.setEnabled(False)
        self.victimz.setEnabled(False)
        self.sensorID.setEnabled(False)
        self.probability.setEnabled(False)
        self.internalGrid.removeWidget(self._ProbabilityInfoWidget)
        self._ProbabilityInfoWidget.shutdown()
        self._ProbabilityInfoWidget.close()

    def setVictimInfo(self):
        self.victimx.setText(" Victim X Position " + str(self._victimInfo[0]))
        self.victimy.setText(" Victim Y Position " + str(self._victimInfo[1]))
        self.victimz.setText(" Victim Z Position " + str(self._victimInfo[2]))
        self.sensorID.setText(" Sensor ID " + "".join(self._victimInfo[4]))
        self.probability.setText(" Probability " + str(self._victimInfo[3]))

    # Refreshing the topics
    @Slot()
    def refresh_topics(self):

        if self.score_info.last_message is not None:
            self.score.display(self.score_info.last_message.data)
        if self.victims_number.last_message is not None:
            self.victimsFound.display(self.victims_number.last_message.data)

        if self.timerStarted:
            self._time = self._time.addSecs(-1)
            self.timer.setTime(self._time)

        # Enable the victim found Options if it is found
        if self.ValidateVictimActionServer._victimFound:
            self._victimInfo = self.ValidateVictimActionServer._victimInfo
            self.enableVictimFoundOptions()

    #Start the timer
    def go_button_clicked(self):

        self.timerStarted = True
        self._time = (self.timer.time())
        self.timer.setTime(self._time)

    #Stop the timer
    def stop_button_clicked(self):

        self.timerStarted = False

    def decline_button_clicked(self):
        self.ValidateVictimActionServer._victimValid = False
        self.disableVictimFoundOptions()
        self.ValidateVictimActionServer._operatorResponded = True
        self.ValidateVictimActionServer._victimFound = False

    def confirm_victim_clicked(self):
        self.ValidateVictimActionServer._victimValid = True
        self.ValidateVictimActionServer._operatorResponded = True
        self.disableVictimFoundOptions()
        self.ValidateVictimActionServer._victimFound = False

    def left_panel_button_clicked(self):
        if self._left_panel:
            self.image.close()
            self.victimInfo_2.close()
            self._left_panel = False
            self.left_panel_button.setText("ShowLeftPanel")
        else :
            self.image.show()
            self.victimInfo_2.show()
            self._left_panel = True
            self.left_panel_button.setText("HideLeftPanel")

    def search_and_rescue_button_clicked(self):
      pass
      
    def mapping_mission_button_clicked(self):
      pass
      
    def teleop_state_button_clicked(self):
      pass
      
    def semi_autonomous_state_clicked(self):
      pass
         
    #The checkboxes slots
    def sonars_checked(self):

        self._sonarsChecked = self.sonarsCheckBox.isChecked()

    def battery_checked(self):

        self._batteryChecked = self.batteryCheckBox.isChecked()

    def temp_checked(self):

        self._tempChecked = self.tempCheckBox.isChecked()

    def co2_checked(self):

        self._co2Checked = self.co2CheckBox.isChecked()

    def show_all(self):
        self._showAllChecked = self.showAllCheckBox.isChecked()

    #Method called when the Widget is terminated
    def shutdown(self):
        self.victims_number.stop_monitoring()
        self.score_info.stop_monitoring()
        self._timer_refresh_widget.stop()
        self.ValidateVictimActionServer.shutdown()
