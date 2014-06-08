import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Slot, QTimer
from python_qt_binding.QtGui import QWidget
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Int16
from pandora_xmega_hardware_interface.msg import BatteryMsg
from .widget_info import WidgetInfo

battery_topic = "/sensors/battery"
class BatteryWidget(QWidget):
    """
    BatteryWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin=None):

        super(BatteryWidget, self).__init__()

        #Load Ui and name the widget
        self._id = "Battery"
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'BatteryWidget.ui')
        loadUi(ui_file, self)

        #create the subcribers
        self.widget_info_PSU = WidgetInfo("chatter", Int16)
        self.widget_info_Motor = WidgetInfo("chatter", Int16)

        #create and connect the timer
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_PSU.start_monitoring()
        self.widget_info_Motor.start_monitoring()
        self._timer_refresh_widget.start(1000)

    #Connected slot to the timer in order to refresh
    @Slot()
    def refresh_topics(self):

        if self.widget_info_PSU.last_message is not None:
            self.lcd1.display(self.widget_info_PSU.last_message.data)
            self.PSUBatteryBar.setValue(self.widget_info_PSU.last_message.data)
        if self.widget_info_Motor.last_message is not None:
            self.lcd2.display(self.widget_info_Motor.last_message.data)
            self.MotorBatteryBar.setValue(self.widget_info_Motor.last_message.data)

    #Method called when the Widget is terminated
    def shutdown(self):
        self.widget_info_PSU.stop_monitoring()
        self.widget_info_Motor.stop_monitoring()
        self._timer_refresh_widget.stop()
