import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from geometry_msgs.msg import Wrench, WrenchStamped

import threading 
 
class PublisherThread(threading.Thread):
    _lock = threading.Lock()
    _isRunning = False
    _isEnd = False
    _wrench = Wrench()
    _rate = rospy.Rate(500) # 500hz
    _topic = "wrench"
    _frameId = "force_frame"

    def __init__(self): 
        threading.Thread.__init__(self) 

    def run(self):
        while not self._isEnd:
            if self._isRunning:
                self.pub = rospy.Publisher(self._topic, WrenchStamped, queue_size=10)
                while self._isRunning:
                    msg = WrenchStamped()
                    msg.header.stamp = rospy.Time.now()
                    self._lock.acquire() 
                    msg.wrench = self._wrench
                    msg.header.frame_id = self._frameId
                    self._lock.release() 
                    self.pub.publish(msg)
                    self._rate.sleep()
            rospy.sleep(0.1)
        print "Thread ended"

    def stop(self):
        self._isRunning = False

    def restart(self):
        self._isRunning = True

    def end(self):
        self._isEnd = True
        self._isRunning = False
        print "Thread ending"

    def setWrenchFX(self, x):
        self._lock.acquire()
        self._wrench.force.x = x
        self._lock.release()
        print self._wrench

    def setWrenchFY(self, y):
        self._lock.acquire()
        self._wrench.force.y = y
        self._lock.release()
        print self._wrench

    def setWrenchFZ(self, z):
        self._lock.acquire()
        self._wrench.force.z = z
        self._lock.release()
        print self._wrench

    def setWrenchTX(self, x):
        self._lock.acquire()
        self._wrench.torque.x = x
        self._lock.release()
        print self._wrench

    def setWrenchTY(self, y):
        self._lock.acquire()
        self._wrench.torque.y = y
        self._lock.release()
        print self._wrench
    
    def setWrenchTZ(self, z):
        self._lock.acquire()
        self._wrench.torque.z  = z
        self._lock.release()
        print self._wrench

    def getWrench(self):
        return self._wrench

    def setRate(self, rate):
        self._rate = rospy.Rate(rate)

    def setTopic(self, topic):
        self._topic = topic

    def setFrameId(self, frameId):
        self._lock.acquire()
        self._frameId = frameId
        self._lock.release()

class kms40_emulator_plugin(Plugin):

    def __init__(self, context):
        super(kms40_emulator_plugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('kms40_emulator_plugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'weiss_kms40_emulator.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('kms40_emulator_pluginUi')

        # Connect UI to slots
        self._widget.pushButton_run.clicked[bool].connect(self._handle_run_clicked)
        self._widget.pushButton_stop.clicked[bool].connect(self._handle_stop_clicked)

        self._widget.spinBox_rate.valueChanged.connect(self._handle_rate_changed)

        self._widget.doubleSpinBox_forceX.valueChanged.connect(self._handle_forceX_changed)
        self._widget.doubleSpinBox_forceY.valueChanged.connect(self._handle_forceY_changed)
        self._widget.doubleSpinBox_forceZ.valueChanged.connect(self._handle_forceZ_changed)
        self._widget.doubleSpinBox_forceA.valueChanged.connect(self._handle_forceA_changed)
        self._widget.doubleSpinBox_forceB.valueChanged.connect(self._handle_forceB_changed)
        self._widget.doubleSpinBox_forceC.valueChanged.connect(self._handle_forceC_changed)

        self._widget.horizontalSlider_forceX.valueChanged.connect(self._handle_sliderX_changed)
        self._widget.horizontalSlider_forceY.valueChanged.connect(self._handle_sliderY_changed)
        self._widget.horizontalSlider_forceZ.valueChanged.connect(self._handle_sliderZ_changed)
        self._widget.horizontalSlider_forceA.valueChanged.connect(self._handle_sliderA_changed)
        self._widget.horizontalSlider_forceB.valueChanged.connect(self._handle_sliderB_changed)
        self._widget.horizontalSlider_forceC.valueChanged.connect(self._handle_sliderC_changed)

        self._widget.lineEdit_topic.editingFinished.connect(self._handle_topic_changed)
        self._widget.lineEdit_frameId.textChanged.connect(self._handle_frameId_changed)

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        # PublisherThread
        self.t = PublisherThread()
        self.t.start()
        self.updateWidgets()

    def shutdown_plugin(self):
        self.t.end()
        self.t.join()
        # TODO unregister all publishers here

        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


    def _handle_run_clicked(self):
        print "run now"
        self.t.restart()


    def _handle_stop_clicked(self):
        print "stop now"
        self.t.stop()


    def _handle_rate_changed(self):
        val = self._widget.spinBox_rate.value()
        #print "handle_rate_changed", int(val)
        self.t.setRate(val)

    #spinBoxes
    def _handle_forceX_changed(self):
        val = self._widget.doubleSpinBox_forceX.value()
        #print "_handle_forceX_changed", val
        self.t.setWrenchFX(val)
        self.updateWidgets()

    def _handle_forceY_changed(self):
        val = self._widget.doubleSpinBox_forceY.value()
        #print "_handle_forceY_changed", val
        self.t.setWrenchFY(val)
        self.updateWidgets()

    def _handle_forceZ_changed(self):
        val = self._widget.doubleSpinBox_forceZ.value()
        #print "_handle_forceZ_changed", val
        self.t.setWrenchFZ(val)
        self.updateWidgets()

    def _handle_forceA_changed(self):
        val = self._widget.doubleSpinBox_forceA.value()
        #print "_handle_forceA_changed", val
        self.t.setWrenchTZ(val)
        self.updateWidgets()

    def _handle_forceB_changed(self):
        val = self._widget.doubleSpinBox_forceB.value()
        #print "_handle_forceB_changed", val
        self.t.setWrenchTY(val)
        self.updateWidgets()

    def _handle_forceC_changed(self):
        val = self._widget.doubleSpinBox_forceC.value()
        #print "_handle_forceC_changed", val
        self.t.setWrenchTX(val)
        self.updateWidgets()

    # sliders
    def _handle_sliderX_changed(self):
        val = self._widget.horizontalSlider_forceX.value()
        #print "_handle_sliderX_changed", val
        self.t.setWrenchFX(val)
        self.updateWidgets()

    def _handle_sliderY_changed(self):
        val = self._widget.horizontalSlider_forceY.value()
        #print "_handle_sliderY_changed", val
        self.t.setWrenchFY(val)
        self.updateWidgets()

    def _handle_sliderZ_changed(self):
        val = self._widget.horizontalSlider_forceZ.value()
        #print "_handle_sliderZ_changed", val
        self.t.setWrenchFZ(val)
        self.updateWidgets()

    def _handle_sliderA_changed(self):
        val = self._widget.horizontalSlider_forceA.value()
        #print "_handle_sliderA_changed", val
        self.t.setWrenchTZ(val)
        self.updateWidgets()

    def _handle_sliderB_changed(self):
        val = self._widget.horizontalSlider_forceB.value()
        #print "_handle_sliderB_changed", val
        self.t.setWrenchTY(val)
        self.updateWidgets()

    def _handle_sliderC_changed(self):
        val = self._widget.horizontalSlider_forceC.value()
        #print "_handle_sliderC_changed", val
        self.t.setWrenchTX(val)
        self.updateWidgets()

    def _handle_frameId_changed(self):
        val = self._widget.lineEdit_frameId.text()
        #print "_handle_topic_changed", val#self._widget.lineEdit_topic.toPlainText()
        self.t.setFrameId(val)

    def _handle_topic_changed(self):
        val = self._widget.lineEdit_topic.text()
        #print "_handle_topic_changed", val#self._widget.lineEdit_topic.toPlainText()
        self.t.setTopic(val)
        self.t.stop()
        rospy.sleep(0.1)
        self.t.restart()

    def updateWidgets(self):
        wrench = self.t.getWrench()
        self._widget.doubleSpinBox_forceX.setValue(wrench.force.x)
        self._widget.doubleSpinBox_forceY.setValue(wrench.force.y)
        self._widget.doubleSpinBox_forceZ.setValue(wrench.force.z)
        self._widget.doubleSpinBox_forceA.setValue(wrench.torque.z)
        self._widget.doubleSpinBox_forceB.setValue(wrench.torque.y)
        self._widget.doubleSpinBox_forceC.setValue(wrench.torque.x)

        self._widget.horizontalSlider_forceX.setValue(wrench.force.x)
        self._widget.horizontalSlider_forceY.setValue(wrench.force.y)
        self._widget.horizontalSlider_forceZ.setValue(wrench.force.z)
        self._widget.horizontalSlider_forceA.setValue(wrench.torque.z)
        self._widget.horizontalSlider_forceB.setValue(wrench.torque.y)
        self._widget.horizontalSlider_forceC.setValue(wrench.torque.x)