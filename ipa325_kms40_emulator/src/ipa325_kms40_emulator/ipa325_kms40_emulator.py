import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

from PyQt4 import QtGui, QtCore

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
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ipa325_kms40_emulator.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('kms40_emulator_pluginUi')

        # Connect UI to slots
        self._widget.pushButton_run.clicked[bool].connect(self._handle_run_clicked)
        self._widget.pushButton_stop.clicked[bool].connect(self._handle_stop_clicked)

        self.connect(self._widget.spinBox_rate, QtCore.SIGNAL("valueChanged(int)"), self._handle_rate_changed)

        self.connect(self._widget.doubleSpinBox_forceX, QtCore.SIGNAL("valueChanged(double)"), self._handle_forceX_changed)
        self.connect(self._widget.doubleSpinBox_forceY, QtCore.SIGNAL("valueChanged(double)"), self._handle_forceY_changed)
        self.connect(self._widget.doubleSpinBox_forceZ, QtCore.SIGNAL("valueChanged(double)"), self._handle_forceZ_changed)
        self.connect(self._widget.doubleSpinBox_forceA, QtCore.SIGNAL("valueChanged(double)"), self._handle_forceA_changed)
        self.connect(self._widget.doubleSpinBox_forceB, QtCore.SIGNAL("valueChanged(double)"), self._handle_forceB_changed)
        self.connect(self._widget.doubleSpinBox_forceC, QtCore.SIGNAL("valueChanged(double)"), self._handle_forceC_changed)

        self.connect(self._widget.horizontalSlider_forceX, QtCore.SIGNAL("valueChanged(int)"), self._handle_sliderX_changed)
        self.connect(self._widget.horizontalSlider_forceY, QtCore.SIGNAL("valueChanged(int)"), self._handle_sliderY_changed)
        self.connect(self._widget.horizontalSlider_forceZ, QtCore.SIGNAL("valueChanged(int)"), self._handle_sliderZ_changed)
        self.connect(self._widget.horizontalSlider_forceA, QtCore.SIGNAL("valueChanged(int)"), self._handle_sliderA_changed)
        self.connect(self._widget.horizontalSlider_forceB, QtCore.SIGNAL("valueChanged(int)"), self._handle_sliderB_changed)
        self.connect(self._widget.horizontalSlider_forceC, QtCore.SIGNAL("valueChanged(int)"), self._handle_sliderC_changed)


        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
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


    def _handle_stop_clicked(self):
        print "stop now"


    def _handle_rate_changed(self, val):
        print "handle_rate_changed", int(val)

    #spinBoxes
    def _handle_forceX_changed(self, val):
        print "_handle_forceX_changed", val

    def _handle_forceY_changed(self, val):
        print "_handle_forceY_changed", val

    def _handle_forceZ_changed(self, val):
        print "_handle_forceZ_changed", val

    def _handle_forceA_changed(self, val):
        print "_handle_forceA_changed", val

    def _handle_forceB_changed(self, val):
        print "_handle_forceB_changed", val

    def _handle_forceC_changed(self, val):
        print "_handle_forceC_changed", val

    # sliders
    def _handle_sliderX_changed(self, val):
        print "_handle_sliderX_changed", val

    def _handle_sliderY_changed(self, val):
        print "_handle_sliderY_changed", val

    def _handle_sliderZ_changed(self, val):
        print "_handle_sliderZ_changed", val

    def _handle_sliderA_changed(self, val):
        print "_handle_sliderA_changed", val

    def _handle_sliderB_changed(self, val):
        print "_handle_sliderB_changed", val

    def _handle_sliderC_changed(self, val):
        print "_handle_sliderC_changed", val