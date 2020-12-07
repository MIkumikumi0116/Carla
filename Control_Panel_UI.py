# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Control_Panel_UI.ui'
#
# Created by: PyQt5 UI code generator 5.15.0
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1453, 872)
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(115, 115, 115))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(115, 115, 115))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(115, 115, 115))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(115, 115, 115))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        MainWindow.setPalette(palette)
        self.Main_Layout = QtWidgets.QWidget(MainWindow)
        self.Main_Layout.setObjectName("Main_Layout")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.Main_Layout)
        self.horizontalLayout.setObjectName("horizontalLayout")
        spacerItem = QtWidgets.QSpacerItem(40, 20,
                                           QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.Evaluation_Layout = QtWidgets.QVBoxLayout()
        self.Evaluation_Layout.setObjectName("Evaluation_Layout")
        self.Evaluation_scrollArea = QtWidgets.QScrollArea(self.Main_Layout)
        self.Evaluation_scrollArea.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.Evaluation_scrollArea.setObjectName("Evaluation_scrollArea")
        self.Evaluation_scrollArea_Widget = QtWidgets.QWidget()
        self.Evaluation_scrollArea_Widget.setGeometry(
            QtCore.QRect(0, 0, 880, 1000))
        self.Evaluation_scrollArea_Widget.setObjectName(
            "Evaluation_scrollArea_Widget")
        self.Evaluatio_GridWidget = QtWidgets.QWidget(
            self.Evaluation_scrollArea_Widget)
        self.Evaluatio_GridWidget.setGeometry(QtCore.QRect(0, 0, 880, 1000))
        self.Evaluatio_GridWidget.setObjectName("Evaluatio_GridWidget")
        self.Evaluation_scrollArea_Widget_GridLayout = QtWidgets.QGridLayout(
            self.Evaluatio_GridWidget)
        self.Evaluation_scrollArea_Widget_GridLayout.setContentsMargins(
            0, 0, 0, 0)
        self.Evaluation_scrollArea_Widget_GridLayout.setSpacing(26)
        self.Evaluation_scrollArea_Widget_GridLayout.setObjectName(
            "Evaluation_scrollArea_Widget_GridLayout")
        self.Stability_Layout = QtWidgets.QVBoxLayout()
        self.Stability_Layout.setObjectName("Stability_Layout")
        self.label_3 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_3.setObjectName("label_3")
        self.Stability_Layout.addWidget(self.label_3)
        self.label_4 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_4.setText("")
        self.label_4.setPixmap(QtGui.QPixmap("blank.png"))
        self.label_4.setScaledContents(True)
        self.label_4.setObjectName("label_4")
        self.Stability_Layout.addWidget(self.label_4)
        self.Stability_Layout.setStretch(1, 1)
        self.Evaluation_scrollArea_Widget_GridLayout.addLayout(
            self.Stability_Layout, 1, 2, 1, 1)
        self.Efficiency_Layout = QtWidgets.QVBoxLayout()
        self.Efficiency_Layout.setObjectName("Efficiency_Layout")
        self.label_5 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_5.setObjectName("label_5")
        self.Efficiency_Layout.addWidget(self.label_5)
        self.label_6 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_6.setText("")
        self.label_6.setPixmap(QtGui.QPixmap("blank.png"))
        self.label_6.setScaledContents(True)
        self.label_6.setObjectName("label_6")
        self.Efficiency_Layout.addWidget(self.label_6)
        self.Efficiency_Layout.setStretch(1, 1)
        self.Evaluation_scrollArea_Widget_GridLayout.addLayout(
            self.Efficiency_Layout, 0, 0, 1, 1)
        self.Overview_Layout = QtWidgets.QVBoxLayout()
        self.Overview_Layout.setObjectName("Overview_Layout")
        self.label_11 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_11.setObjectName("label_11")
        self.Overview_Layout.addWidget(self.label_11)
        self.label_12 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_12.setText("")
        self.label_12.setPixmap(QtGui.QPixmap("blank.png"))
        self.label_12.setScaledContents(True)
        self.label_12.setObjectName("label_12")
        self.Overview_Layout.addWidget(self.label_12)
        self.Overview_Layout.setStretch(1, 1)
        self.Evaluation_scrollArea_Widget_GridLayout.addLayout(
            self.Overview_Layout, 2, 2, 1, 1)
        self.Conflict_Layout = QtWidgets.QVBoxLayout()
        self.Conflict_Layout.setObjectName("Conflict_Layout")
        self.label_7 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_7.setObjectName("label_7")
        self.Conflict_Layout.addWidget(self.label_7)
        self.label_8 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_8.setText("")
        self.label_8.setPixmap(QtGui.QPixmap("blank.png"))
        self.label_8.setScaledContents(True)
        self.label_8.setObjectName("label_8")
        self.Conflict_Layout.addWidget(self.label_8)
        self.Conflict_Layout.setStretch(1, 1)
        self.Evaluation_scrollArea_Widget_GridLayout.addLayout(
            self.Conflict_Layout, 1, 0, 1, 1)
        self.Service_Level_Layout = QtWidgets.QVBoxLayout()
        self.Service_Level_Layout.setObjectName("Service_Level_Layout")
        self.label_9 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_9.setObjectName("label_9")
        self.Service_Level_Layout.addWidget(self.label_9)
        self.label_10 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_10.setText("")
        self.label_10.setPixmap(QtGui.QPixmap("blank.png"))
        self.label_10.setScaledContents(True)
        self.label_10.setObjectName("label_10")
        self.Service_Level_Layout.addWidget(self.label_10)
        self.Service_Level_Layout.setStretch(1, 1)
        self.Evaluation_scrollArea_Widget_GridLayout.addLayout(
            self.Service_Level_Layout, 2, 0, 1, 1)
        self.Safety_Layout = QtWidgets.QVBoxLayout()
        self.Safety_Layout.setObjectName("Safety_Layout")
        self.label = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label.setObjectName("label")
        self.Safety_Layout.addWidget(self.label)
        self.label_2 = QtWidgets.QLabel(self.Evaluatio_GridWidget)
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap("blank.png"))
        self.label_2.setScaledContents(True)
        self.label_2.setObjectName("label_2")
        self.Safety_Layout.addWidget(self.label_2)
        self.Safety_Layout.setStretch(1, 1)
        self.Evaluation_scrollArea_Widget_GridLayout.addLayout(
            self.Safety_Layout, 0, 2, 1, 1)
        self.Evaluation_scrollArea.setWidget(self.Evaluation_scrollArea_Widget)
        self.Evaluation_Layout.addWidget(self.Evaluation_scrollArea)
        self.horizontalLayout.addLayout(self.Evaluation_Layout)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.Control_Panel_Layout = QtWidgets.QVBoxLayout()
        self.Control_Panel_Layout.setObjectName("Control_Panel_Layout")
        spacerItem2 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Minimum,
                                            QtWidgets.QSizePolicy.Expanding)
        self.Control_Panel_Layout.addItem(spacerItem2)
        self.Working_Status_Label = QtWidgets.QLabel(self.Main_Layout)
        self.Working_Status_Label.setText("")
        self.Working_Status_Label.setTextFormat(QtCore.Qt.PlainText)
        self.Working_Status_Label.setAlignment(QtCore.Qt.AlignCenter)
        self.Working_Status_Label.setObjectName("Working_Status_Label")
        self.Control_Panel_Layout.addWidget(self.Working_Status_Label)
        spacerItem3 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Minimum,
                                            QtWidgets.QSizePolicy.Expanding)
        self.Control_Panel_Layout.addItem(spacerItem3)
        self.Setting_Layout = QtWidgets.QVBoxLayout()
        self.Setting_Layout.setSpacing(15)
        self.Setting_Layout.setObjectName("Setting_Layout")
        self.Main_Lane_Triffic_Layout = QtWidgets.QHBoxLayout()
        self.Main_Lane_Triffic_Layout.setObjectName("Main_Lane_Triffic_Layout")
        spacerItem4 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.Main_Lane_Triffic_Layout.addItem(spacerItem4)
        self.Main_Lane_Triffic_Lable = QtWidgets.QLabel(self.Main_Layout)
        self.Main_Lane_Triffic_Lable.setObjectName("Main_Lane_Triffic_Lable")
        self.Main_Lane_Triffic_Layout.addWidget(self.Main_Lane_Triffic_Lable)
        self.Main_Lane_Triffic_Scrollbar = QtWidgets.QScrollBar(
            self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Main_Lane_Triffic_Scrollbar.sizePolicy().hasHeightForWidth())
        self.Main_Lane_Triffic_Scrollbar.setSizePolicy(sizePolicy)
        self.Main_Lane_Triffic_Scrollbar.setMaximum(100)
        self.Main_Lane_Triffic_Scrollbar.setOrientation(QtCore.Qt.Horizontal)
        self.Main_Lane_Triffic_Scrollbar.setObjectName(
            "Main_Lane_Triffic_Scrollbar")
        self.Main_Lane_Triffic_Layout.addWidget(
            self.Main_Lane_Triffic_Scrollbar)
        self.Main_Lane_Triffic_LineEdit = QtWidgets.QLineEdit(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Main_Lane_Triffic_LineEdit.sizePolicy().hasHeightForWidth())
        self.Main_Lane_Triffic_LineEdit.setSizePolicy(sizePolicy)
        self.Main_Lane_Triffic_LineEdit.setStyleSheet(
            "color:white;\n"
            "background-color:rgb(70 , 70 , 70);\n"
            "border: 1px solid rgb(28 , 28 , 28);")
        self.Main_Lane_Triffic_LineEdit.setObjectName(
            "Main_Lane_Triffic_LineEdit")
        self.Main_Lane_Triffic_Layout.addWidget(
            self.Main_Lane_Triffic_LineEdit)
        spacerItem5 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.Main_Lane_Triffic_Layout.addItem(spacerItem5)
        self.Main_Lane_Triffic_Layout.setStretch(0, 1)
        self.Main_Lane_Triffic_Layout.setStretch(4, 1)
        self.Setting_Layout.addLayout(self.Main_Lane_Triffic_Layout)
        self.Ramp_Lane_Triffic_Layout = QtWidgets.QHBoxLayout()
        self.Ramp_Lane_Triffic_Layout.setObjectName("Ramp_Lane_Triffic_Layout")
        spacerItem6 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.Ramp_Lane_Triffic_Layout.addItem(spacerItem6)
        self.Ramp_Lane_Triffic_Lable = QtWidgets.QLabel(self.Main_Layout)
        self.Ramp_Lane_Triffic_Lable.setObjectName("Ramp_Lane_Triffic_Lable")
        self.Ramp_Lane_Triffic_Layout.addWidget(self.Ramp_Lane_Triffic_Lable)
        self.Ramp_Lane_Triffic_Scrollbar = QtWidgets.QScrollBar(
            self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Ramp_Lane_Triffic_Scrollbar.sizePolicy().hasHeightForWidth())
        self.Ramp_Lane_Triffic_Scrollbar.setSizePolicy(sizePolicy)
        self.Ramp_Lane_Triffic_Scrollbar.setMaximum(100)
        self.Ramp_Lane_Triffic_Scrollbar.setOrientation(QtCore.Qt.Horizontal)
        self.Ramp_Lane_Triffic_Scrollbar.setObjectName(
            "Ramp_Lane_Triffic_Scrollbar")
        self.Ramp_Lane_Triffic_Layout.addWidget(
            self.Ramp_Lane_Triffic_Scrollbar)
        self.Ramp_Lane_Triffic_LineEdit = QtWidgets.QLineEdit(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Ramp_Lane_Triffic_LineEdit.sizePolicy().hasHeightForWidth())
        self.Ramp_Lane_Triffic_LineEdit.setSizePolicy(sizePolicy)
        self.Ramp_Lane_Triffic_LineEdit.setStyleSheet(
            "color:white;\n"
            "background-color:rgb(70 , 70 , 70);\n"
            "border: 1px solid rgb(28 , 28 , 28);")
        self.Ramp_Lane_Triffic_LineEdit.setObjectName(
            "Ramp_Lane_Triffic_LineEdit")
        self.Ramp_Lane_Triffic_Layout.addWidget(
            self.Ramp_Lane_Triffic_LineEdit)
        spacerItem7 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.Ramp_Lane_Triffic_Layout.addItem(spacerItem7)
        self.Ramp_Lane_Triffic_Layout.setStretch(0, 1)
        self.Ramp_Lane_Triffic_Layout.setStretch(4, 1)
        self.Setting_Layout.addLayout(self.Ramp_Lane_Triffic_Layout)
        self.Truck_Mixing_Rate_Layout = QtWidgets.QHBoxLayout()
        self.Truck_Mixing_Rate_Layout.setObjectName("Truck_Mixing_Rate_Layout")
        spacerItem8 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.Truck_Mixing_Rate_Layout.addItem(spacerItem8)
        self.Truck_Mixing_Rate_Lable = QtWidgets.QLabel(self.Main_Layout)
        self.Truck_Mixing_Rate_Lable.setObjectName("Truck_Mixing_Rate_Lable")
        self.Truck_Mixing_Rate_Layout.addWidget(self.Truck_Mixing_Rate_Lable)
        self.Truck_Mixing_Rate_Scrollbar = QtWidgets.QScrollBar(
            self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Truck_Mixing_Rate_Scrollbar.sizePolicy().hasHeightForWidth())
        self.Truck_Mixing_Rate_Scrollbar.setSizePolicy(sizePolicy)
        self.Truck_Mixing_Rate_Scrollbar.setMaximum(100)
        self.Truck_Mixing_Rate_Scrollbar.setOrientation(QtCore.Qt.Horizontal)
        self.Truck_Mixing_Rate_Scrollbar.setObjectName(
            "Truck_Mixing_Rate_Scrollbar")
        self.Truck_Mixing_Rate_Layout.addWidget(
            self.Truck_Mixing_Rate_Scrollbar)
        self.Truck_Mixing_Rate_LineEdit = QtWidgets.QLineEdit(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Truck_Mixing_Rate_LineEdit.sizePolicy().hasHeightForWidth())
        self.Truck_Mixing_Rate_LineEdit.setSizePolicy(sizePolicy)
        self.Truck_Mixing_Rate_LineEdit.setStyleSheet(
            "color:white;\n"
            "background-color:rgb(70 , 70 , 70);\n"
            "border: 1px solid rgb(28 , 28 , 28);")
        self.Truck_Mixing_Rate_LineEdit.setObjectName(
            "Truck_Mixing_Rate_LineEdit")
        self.Truck_Mixing_Rate_Layout.addWidget(
            self.Truck_Mixing_Rate_LineEdit)
        spacerItem9 = QtWidgets.QSpacerItem(0, 0,
                                            QtWidgets.QSizePolicy.Expanding,
                                            QtWidgets.QSizePolicy.Minimum)
        self.Truck_Mixing_Rate_Layout.addItem(spacerItem9)
        self.Truck_Mixing_Rate_Layout.setStretch(0, 1)
        self.Truck_Mixing_Rate_Layout.setStretch(4, 1)
        self.Setting_Layout.addLayout(self.Truck_Mixing_Rate_Layout)
        self.Speed_Setting_Layout = QtWidgets.QHBoxLayout()
        self.Speed_Setting_Layout.setObjectName("Speed_Setting_Layout")
        spacerItem10 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Speed_Setting_Layout.addItem(spacerItem10)
        self.Speed_Setting_Lable = QtWidgets.QLabel(self.Main_Layout)
        self.Speed_Setting_Lable.setObjectName("Speed_Setting_Lable")
        self.Speed_Setting_Layout.addWidget(self.Speed_Setting_Lable)
        self.Speed_Setting_Scrollbar = QtWidgets.QScrollBar(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Speed_Setting_Scrollbar.sizePolicy().hasHeightForWidth())
        self.Speed_Setting_Scrollbar.setSizePolicy(sizePolicy)
        self.Speed_Setting_Scrollbar.setMaximum(300)
        self.Speed_Setting_Scrollbar.setOrientation(QtCore.Qt.Horizontal)
        self.Speed_Setting_Scrollbar.setObjectName("Speed_Setting_Scrollbar")
        self.Speed_Setting_Layout.addWidget(self.Speed_Setting_Scrollbar)
        self.Speed_Setting_LineEdit = QtWidgets.QLineEdit(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Speed_Setting_LineEdit.sizePolicy().hasHeightForWidth())
        self.Speed_Setting_LineEdit.setSizePolicy(sizePolicy)
        self.Speed_Setting_LineEdit.setStyleSheet(
            "color:white;\n"
            "background-color:rgb(70 , 70 , 70);\n"
            "border: 1px solid rgb(28 , 28 , 28);")
        self.Speed_Setting_LineEdit.setObjectName("Speed_Setting_LineEdit")
        self.Speed_Setting_Layout.addWidget(self.Speed_Setting_LineEdit)
        spacerItem11 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Speed_Setting_Layout.addItem(spacerItem11)
        self.Speed_Setting_Layout.setStretch(0, 1)
        self.Speed_Setting_Layout.setStretch(4, 1)
        self.Setting_Layout.addLayout(self.Speed_Setting_Layout)
        self.Auto_Driving_Rate_Layout = QtWidgets.QHBoxLayout()
        self.Auto_Driving_Rate_Layout.setContentsMargins(0, -1, 0, -1)
        self.Auto_Driving_Rate_Layout.setObjectName("Auto_Driving_Rate_Layout")
        spacerItem12 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Auto_Driving_Rate_Layout.addItem(spacerItem12)
        self.Auto_Driving_Rate_Lable = QtWidgets.QLabel(self.Main_Layout)
        self.Auto_Driving_Rate_Lable.setObjectName("Auto_Driving_Rate_Lable")
        self.Auto_Driving_Rate_Layout.addWidget(self.Auto_Driving_Rate_Lable)
        self.Auto_Driving_Rate_Scrollbar = QtWidgets.QScrollBar(
            self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Auto_Driving_Rate_Scrollbar.sizePolicy().hasHeightForWidth())
        self.Auto_Driving_Rate_Scrollbar.setSizePolicy(sizePolicy)
        self.Auto_Driving_Rate_Scrollbar.setMaximum(100)
        self.Auto_Driving_Rate_Scrollbar.setOrientation(QtCore.Qt.Horizontal)
        self.Auto_Driving_Rate_Scrollbar.setObjectName(
            "Auto_Driving_Rate_Scrollbar")
        self.Auto_Driving_Rate_Layout.addWidget(
            self.Auto_Driving_Rate_Scrollbar)
        self.Auto_Driving_Rate_LineEdit = QtWidgets.QLineEdit(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Auto_Driving_Rate_LineEdit.sizePolicy().hasHeightForWidth())
        self.Auto_Driving_Rate_LineEdit.setSizePolicy(sizePolicy)
        self.Auto_Driving_Rate_LineEdit.setStyleSheet(
            "color:white;\n"
            "background-color:rgb(70 , 70 , 70);\n"
            "border: 1px solid rgb(28 , 28 , 28);")
        self.Auto_Driving_Rate_LineEdit.setObjectName(
            "Auto_Driving_Rate_LineEdit")
        self.Auto_Driving_Rate_Layout.addWidget(
            self.Auto_Driving_Rate_LineEdit)
        spacerItem13 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Auto_Driving_Rate_Layout.addItem(spacerItem13)
        self.Auto_Driving_Rate_Layout.setStretch(0, 1)
        self.Auto_Driving_Rate_Layout.setStretch(4, 1)
        self.Setting_Layout.addLayout(self.Auto_Driving_Rate_Layout)
        self.Simulation_Stride_Layout = QtWidgets.QHBoxLayout()
        self.Simulation_Stride_Layout.setObjectName("Simulation_Stride_Layout")
        spacerItem14 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Simulation_Stride_Layout.addItem(spacerItem14)
        self.Simulation_Stride_Lable = QtWidgets.QLabel(self.Main_Layout)
        self.Simulation_Stride_Lable.setObjectName("Simulation_Stride_Lable")
        self.Simulation_Stride_Layout.addWidget(self.Simulation_Stride_Lable)
        self.Simulation_Stride_Scrollbar = QtWidgets.QScrollBar(
            self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Simulation_Stride_Scrollbar.sizePolicy().hasHeightForWidth())
        self.Simulation_Stride_Scrollbar.setSizePolicy(sizePolicy)
        self.Simulation_Stride_Scrollbar.setMinimum(1)
        self.Simulation_Stride_Scrollbar.setMaximum(100)
        self.Simulation_Stride_Scrollbar.setOrientation(QtCore.Qt.Horizontal)
        self.Simulation_Stride_Scrollbar.setObjectName(
            "Simulation_Stride_Scrollbar")
        self.Simulation_Stride_Layout.addWidget(
            self.Simulation_Stride_Scrollbar)
        self.Simulation_Stride_LineEdit = QtWidgets.QLineEdit(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Simulation_Stride_LineEdit.sizePolicy().hasHeightForWidth())
        self.Simulation_Stride_LineEdit.setSizePolicy(sizePolicy)
        self.Simulation_Stride_LineEdit.setStyleSheet(
            "color:white;\n"
            "background-color:rgb(70 , 70 , 70);\n"
            "border: 1px solid rgb(28 , 28 , 28);")
        self.Simulation_Stride_LineEdit.setObjectName(
            "Simulation_Stride_LineEdit")
        self.Simulation_Stride_Layout.addWidget(
            self.Simulation_Stride_LineEdit)
        spacerItem15 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Simulation_Stride_Layout.addItem(spacerItem15)
        self.Simulation_Stride_Layout.setStretch(0, 1)
        self.Simulation_Stride_Layout.setStretch(4, 1)
        self.Setting_Layout.addLayout(self.Simulation_Stride_Layout)
        self.Control_Panel_Layout.addLayout(self.Setting_Layout)
        spacerItem16 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Minimum,
                                             QtWidgets.QSizePolicy.Expanding)
        self.Control_Panel_Layout.addItem(spacerItem16)
        self.RBotton_Layout = QtWidgets.QHBoxLayout()
        self.RBotton_Layout.setObjectName("RBotton_Layout")
        spacerItem17 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.RBotton_Layout.addItem(spacerItem17)
        self.Following_Model_RadioB = QtWidgets.QRadioButton(self.Main_Layout)
        self.Following_Model_RadioB.setChecked(True)
        self.Following_Model_RadioB.setObjectName("Following_Model_RadioB")
        self.RBotton_Layout.addWidget(self.Following_Model_RadioB)
        self.Lane_Change_Model_RadioB = QtWidgets.QRadioButton(
            self.Main_Layout)
        self.Lane_Change_Model_RadioB.setObjectName("Lane_Change_Model_RadioB")
        self.RBotton_Layout.addWidget(self.Lane_Change_Model_RadioB)
        spacerItem18 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.RBotton_Layout.addItem(spacerItem18)
        self.RBotton_Layout.setStretch(0, 10)
        self.RBotton_Layout.setStretch(3, 10)
        self.Control_Panel_Layout.addLayout(self.RBotton_Layout)
        spacerItem19 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Minimum,
                                             QtWidgets.QSizePolicy.Expanding)
        self.Control_Panel_Layout.addItem(spacerItem19)
        self.Botton_Layout = QtWidgets.QHBoxLayout()
        self.Botton_Layout.setObjectName("Botton_Layout")
        spacerItem20 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Botton_Layout.addItem(spacerItem20)
        self.Botton_Layout_2 = QtWidgets.QVBoxLayout()
        self.Botton_Layout_2.setSpacing(40)
        self.Botton_Layout_2.setObjectName("Botton_Layout_2")
        self.New_Simulation_Button = QtWidgets.QPushButton(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.New_Simulation_Button.sizePolicy().hasHeightForWidth())
        self.New_Simulation_Button.setSizePolicy(sizePolicy)
        self.New_Simulation_Button.setMinimumSize(QtCore.QSize(0, 25))
        self.New_Simulation_Button.setStyleSheet(
            "QPushButton{\n"
            "color:white;\n"
            "background-color:rgb(65 , 65 , 65);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:hover{\n"
            "color:white;\n"
            "background-color:rgb(75 , 75 , 75);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:pressed{\n"
            "color:white;\n"
            "background-color:rgb(85 , 85 , 85);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:disabled{\n"
            "color:white;\n"
            "background-color:rgb(95 , 95 , 95);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}")
        self.New_Simulation_Button.setObjectName("New_Simulation_Button")
        self.Botton_Layout_2.addWidget(self.New_Simulation_Button)
        self.Pause_Simulation_Button = QtWidgets.QPushButton(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Pause_Simulation_Button.sizePolicy().hasHeightForWidth())
        self.Pause_Simulation_Button.setSizePolicy(sizePolicy)
        self.Pause_Simulation_Button.setMinimumSize(QtCore.QSize(0, 25))
        self.Pause_Simulation_Button.setStyleSheet(
            "QPushButton{\n"
            "color:white;\n"
            "background-color:rgb(65 , 65 , 65);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:hover{\n"
            "color:white;\n"
            "background-color:rgb(75 , 75 , 75);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:pressed{\n"
            "color:white;\n"
            "background-color:rgb(85 , 85 , 85);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:disabled{\n"
            "color:white;\n"
            "background-color:rgb(95 , 95 , 95);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}")
        self.Pause_Simulation_Button.setObjectName("Pause_Simulation_Button")
        self.Botton_Layout_2.addWidget(self.Pause_Simulation_Button)
        self.End_Simulation_Button = QtWidgets.QPushButton(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.End_Simulation_Button.sizePolicy().hasHeightForWidth())
        self.End_Simulation_Button.setSizePolicy(sizePolicy)
        self.End_Simulation_Button.setMinimumSize(QtCore.QSize(0, 25))
        self.End_Simulation_Button.setStyleSheet(
            "QPushButton{\n"
            "color:white;\n"
            "background-color:rgb(65 , 65 , 65);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:hover{\n"
            "color:white;\n"
            "background-color:rgb(75 , 75 , 75);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:pressed{\n"
            "color:white;\n"
            "background-color:rgb(85 , 85 , 85);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:disabled{\n"
            "color:white;\n"
            "background-color:rgb(95 , 95 , 95);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}")
        self.End_Simulation_Button.setObjectName("End_Simulation_Button")
        self.Botton_Layout_2.addWidget(self.End_Simulation_Button)
        self.Export_Data_Button = QtWidgets.QPushButton(self.Main_Layout)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum,
                                           QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(
            self.Export_Data_Button.sizePolicy().hasHeightForWidth())
        self.Export_Data_Button.setSizePolicy(sizePolicy)
        self.Export_Data_Button.setMinimumSize(QtCore.QSize(0, 25))
        self.Export_Data_Button.setStyleSheet(
            "QPushButton{\n"
            "color:white;\n"
            "background-color:rgb(65 , 65 , 65);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:hover{\n"
            "color:white;\n"
            "background-color:rgb(75 , 75 , 75);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:pressed{\n"
            "color:white;\n"
            "background-color:rgb(85 , 85 , 85);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}\n"
            "QPushButton:disabled{\n"
            "color:white;\n"
            "background-color:rgb(95 , 95 , 95);\n"
            "border: 1px solid rgb(28 , 28 , 28);\n"
            "}")
        self.Export_Data_Button.setObjectName("Export_Data_Button")
        self.Botton_Layout_2.addWidget(self.Export_Data_Button)
        self.Botton_Layout.addLayout(self.Botton_Layout_2)
        spacerItem21 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.Botton_Layout.addItem(spacerItem21)
        self.Botton_Layout.setStretch(0, 1)
        self.Botton_Layout.setStretch(1, 3)
        self.Botton_Layout.setStretch(2, 1)
        self.Control_Panel_Layout.addLayout(self.Botton_Layout)
        spacerItem22 = QtWidgets.QSpacerItem(0, 0,
                                             QtWidgets.QSizePolicy.Minimum,
                                             QtWidgets.QSizePolicy.Expanding)
        self.Control_Panel_Layout.addItem(spacerItem22)
        self.Control_Panel_Layout.setStretch(0, 1)
        self.Control_Panel_Layout.setStretch(2, 1)
        self.Control_Panel_Layout.setStretch(4, 1)
        self.Control_Panel_Layout.setStretch(6, 1)
        self.Control_Panel_Layout.setStretch(8, 2)
        self.horizontalLayout.addLayout(self.Control_Panel_Layout)
        spacerItem23 = QtWidgets.QSpacerItem(40, 20,
                                             QtWidgets.QSizePolicy.Expanding,
                                             QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem23)
        self.horizontalLayout.setStretch(1, 10)
        self.horizontalLayout.setStretch(3, 4)
        MainWindow.setCentralWidget(self.Main_Layout)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "控制面板"))
        self.label_3.setText(_translate("MainWindow", "稳定性评价"))
        self.label_5.setText(_translate("MainWindow", "效率评价"))
        self.label_11.setText(_translate("MainWindow", "综合评价"))
        self.label_7.setText(_translate("MainWindow", "冲突评价"))
        self.label_9.setText(_translate("MainWindow", "服务水平"))
        self.label.setText(_translate("MainWindow", "安全评价"))
        self.Main_Lane_Triffic_Lable.setText(
            _translate("MainWindow", "主线交通量  "))
        self.Main_Lane_Triffic_LineEdit.setText(_translate("MainWindow", "0"))
        self.Ramp_Lane_Triffic_Lable.setText(
            _translate("MainWindow", "匝道交通量  "))
        self.Ramp_Lane_Triffic_LineEdit.setText(_translate("MainWindow", "0"))
        self.Truck_Mixing_Rate_Lable.setText(
            _translate("MainWindow", "大车混入率  "))
        self.Truck_Mixing_Rate_LineEdit.setText(_translate("MainWindow", "0"))
        self.Speed_Setting_Lable.setText(_translate("MainWindow", "速度设定    "))
        self.Speed_Setting_LineEdit.setText(_translate("MainWindow", "0"))
        self.Auto_Driving_Rate_Lable.setText(_translate(
            "MainWindow", "自动驾驶比率"))
        self.Auto_Driving_Rate_LineEdit.setText(_translate("MainWindow", "0"))
        self.Simulation_Stride_Lable.setText(
            _translate("MainWindow", "仿真步长    "))
        self.Simulation_Stride_LineEdit.setText(_translate("MainWindow", "1"))
        self.Following_Model_RadioB.setText(_translate("MainWindow", "跟驰模型"))
        self.Lane_Change_Model_RadioB.setText(_translate("MainWindow", "换道模型"))
        self.New_Simulation_Button.setText(_translate("MainWindow", "开始新的仿真"))
        self.Pause_Simulation_Button.setText(_translate("MainWindow", "暂停仿真"))
        self.End_Simulation_Button.setText(_translate("MainWindow", "结束仿真"))
        self.Export_Data_Button.setText(_translate("MainWindow", "导出数据"))
