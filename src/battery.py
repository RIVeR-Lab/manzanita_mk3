import roslib;roslib.load_manifest('manzanita_mk3_driver')
import os
import rospy
import manzanita_mk3_driver.msg

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Signal, Qt
from python_qt_binding import loadUi, QtGui, QtCore

VOLTAGE_WARNING_INTERVAL = 5*60
VOLTAGE_CRITICAL_INTERVAL = 2*60

class BatteryDashboard(Plugin):
   state_changed = Signal(manzanita_mk3_driver.msg.BatteryPack)
   def __init__(self, context):
      super(BatteryDashboard, self).__init__(context)
      self.setObjectName('BatteryDashboard')

      self._widget = QtGui.QWidget()
      self._widget.setObjectName('BatteryDashboardUi')
      self._widget.setWindowTitle('MK3 Battery Dashboard')
      self.vbox = QtGui.QVBoxLayout()
      self._widget.setLayout(self.vbox)
      self.state_changed.connect(self._update_state)
      
      self.totalBar = QtGui.QProgressBar()
      self.totalBar.setValue(0)
      self.totalBar.setFormat('Waiting for message')
      self.vbox.addWidget(self.totalBar)

      self.last_voltage_warning = rospy.get_rostime()-rospy.Duration(VOLTAGE_WARNING_INTERVAL)
      self.last_voltage_critical = rospy.get_rostime()-rospy.Duration(VOLTAGE_CRITICAL_INTERVAL)



      self._battery_sub = rospy.Subscriber('/battery', manzanita_mk3_driver.msg.BatteryPack, self.battery_callback)
      context.add_widget(self._widget)

   inited = False
   def _update_state(self, msg):
      self.totalBar.setValue(100*msg.total_voltage/msg.max_total_voltage)
      self.totalBar.setFormat(str(msg.total_voltage)+' V')

      if not self.inited:
         cellContainer = QtGui.QWidget()
         cellLayout = QtGui.QFormLayout()
         cellContainer.setLayout(cellLayout)
         i = 0
         self.cellBars = list()
         for cell in msg.cells:
            voltBar = QtGui.QProgressBar()
            voltBar.setGeometry(0, 0, 150, 25)
            voltBar.setValue(75)
            voltBar.setFormat('Cell '+str(cell.cell_id))
            cellLayout.addRow(cell.frame_id, voltBar)
            self.cellBars.append(voltBar)
            i+=1
         self.vbox.addWidget(cellContainer)
         
         self.alert = QtGui.QMessageBox(QtGui.QMessageBox.Information, 'Battery Voltage', 'The current battery voltage is: '+str(msg.total_voltage)+' V', flags=Qt.Dialog|Qt.MSWindowsFixedSizeDialogHint|Qt.WindowStaysOnTopHint)
         self.alert.show()

         self.inited = True

      i = 0
      for cell in msg.cells:
         self.cellBars[i].setValue(100*cell.voltage/msg.max_cell_voltage)
         self.cellBars[i].setFormat(str(cell.voltage)+' V')
         i+=1

      if msg.total_voltage<msg.critical_total_voltage:
         if (rospy.get_rostime()-self.last_voltage_critical) > rospy.Duration(VOLTAGE_CRITICAL_INTERVAL):
            self.alert = QtGui.QMessageBox(QtGui.QMessageBox.Warning, 'Battery Voltage Critical!!!', 'The current battery voltage is: '+str(msg.total_voltage)+' V. You should shutdown now!!!', flags=Qt.Dialog|Qt.MSWindowsFixedSizeDialogHint|Qt.WindowStaysOnTopHint)
            self.alert.show()
            self.last_voltage_critical = rospy.get_rostime()

      elif msg.total_voltage<msg.warn_total_voltage:
         if (rospy.get_rostime()-self.last_voltage_warning) > rospy.Duration(VOLTAGE_WARNING_INTERVAL):
            self.alert = QtGui.QMessageBox(QtGui.QMessageBox.Warning, 'Battery Voltage Low', 'The current battery voltage is: '+str(msg.total_voltage)+' V. You should plug in soon', flags=Qt.Dialog|Qt.MSWindowsFixedSizeDialogHint|Qt.WindowStaysOnTopHint)
            self.alert.show()
            self.last_voltage_warning = rospy.get_rostime()
      




   def shutdown_plugin(self):
      self._battery_sub.unregister()
      pass

   def save_settings(self, plugin_settings, instance_settings):
      pass

   def restore_settings(self, plugin_settings, instance_settings):
      pass


   def battery_callback(self, msg):
      self.state_changed.emit(msg)

