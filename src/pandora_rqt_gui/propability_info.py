from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QHeaderView, QIcon, QMenu, QTreeWidgetItem, QWidget
import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Int16

from data_fusion_communications.msg import FusionGlobalMsg



global_propabilities_topic= "/data_fusion/victim_fusion/global_probabilities"



from .widget_info import WidgetInfo


class PropabilityInfoWidget(QWidget):
    """
   PropabilityInfoWidget.start must be called in order to update topic pane.
    """

    def __init__(self, plugin = None ):
       
       
        super(PropabilityInfoWidget, self).__init__()
        
        self._id= "PropabilityInfo"
        
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('pandora_rqt_gui'), 'resources', 'PropabilityInfo.ui')
        loadUi(ui_file, self)
        self.widget_info_CO2 = WidgetInfo(global_propabilities_topic, FusionGlobalMsg )
        self.widget_info_Thermal = WidgetInfo(global_propabilities_topic, FusionGlobalMsg )
        self.widget_info_Motion = WidgetInfo(global_propabilities_topic, FusionGlobalMsg )
        self.widget_info_Sound = WidgetInfo(global_propabilities_topic, FusionGlobalMsg )
        self.widget_info_Face = WidgetInfo(global_propabilities_topic, FusionGlobalMsg )

        
        self._timer_refresh_widget = QTimer(self)
        self._timer_refresh_widget.timeout.connect(self.refresh_topics)

    def start(self):
        self.widget_info_CO2.start_monitoring()
        self.widget_info_Thermal.start_monitoring()
        self.widget_info_Motion.start_monitoring()
        self.widget_info_Sound.start_monitoring()
        self.widget_info_Face.start_monitoring()
        self._timer_refresh_widget.start(500)

    @Slot()
    def refresh_topics(self):

       if self.widget_info_CO2.last_message is not None:
         self.co2Bar.setValue(self.widget_info_CO2.last_message.co2 % 100 )
         self.thermalBar.setValue(self.widget_info_Thermal.last_message.mlx % 100)
         self.motionBar.setValue(self.widget_info_Motion.last_message.motion % 100)
         self.soundBar.setValue(self.widget_info_Sound.last_message.sound % 100)
         self.faceBar.setValue(self.widget_info_Face.last_message.face % 100)


    def shutdown(self):
        self.widget_info_CO2.stop_monitoring()
        self.widget_info_Thermal.stop_monitoring()
        self.widget_info_Motion.stop_monitoring()
        self.widget_info_Sound.stop_monitoring()
        self.widget_info_Face.stop_monitoring()
        self._timer_refresh_widget.stop()
       
  
      
