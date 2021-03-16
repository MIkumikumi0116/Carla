# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'f:\Code\Carla\Carla2.0\Main_Window_UI.ui'
#
# Created by: PyQt5 UI code generator 5.15.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Main_Window_UI(object):
    def setupUi(self, Main_Window_UI):
        Main_Window_UI.setObjectName("Main_Window_UI")
        Main_Window_UI.resize(1200, 800)
        Main_Window_UI.setStyleSheet("#Main_Window_UI{\n"
"    background: rgb(44,42,56);\n"
"}")
        self.Central_Widget = QtWidgets.QWidget(Main_Window_UI)
        self.Central_Widget.setObjectName("Central_Widget")
        self.Central_Widget1 = QtWidgets.QVBoxLayout(self.Central_Widget)
        self.Central_Widget1.setContentsMargins(0, 0, 0, 0)
        self.Central_Widget1.setSpacing(0)
        self.Central_Widget1.setObjectName("Central_Widget1")
        self.Title_Widget = QtWidgets.QWidget(self.Central_Widget)
        self.Title_Widget.setStyleSheet("#Title_Widget{\n"
"    background: rgb(24,22,36);\n"
"}\n"
"\n"
"#Title_Widget QPushButton{\n"
"    border: 0px;\n"
"\n"
"    width: 50px;\n"
"    height: 30px;\n"
"\n"
"    background: rgb(24,22,36);\n"
"}\n"
"#Title_Widget QPushButton:hover{\n"
"    background: rgb(64,62,76);\n"
"}\n"
"#Title_Widget QPushButton:pressed{\n"
"    background: rgb(84,82,96);\n"
"}")
        self.Title_Widget.setObjectName("Title_Widget")
        self.Title_Layout = QtWidgets.QHBoxLayout(self.Title_Widget)
        self.Title_Layout.setContentsMargins(0, 0, 0, 0)
        self.Title_Layout.setSpacing(0)
        self.Title_Layout.setObjectName("Title_Layout")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.Title_Layout.addItem(spacerItem)
        self.Minimsize_Button = QtWidgets.QPushButton(self.Title_Widget)
        self.Minimsize_Button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/All_Image/res/Minimsize.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Minimsize_Button.setIcon(icon)
        self.Minimsize_Button.setObjectName("Minimsize_Button")
        self.Title_Layout.addWidget(self.Minimsize_Button)
        self.Maxmsize_Button = QtWidgets.QPushButton(self.Title_Widget)
        self.Maxmsize_Button.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(":/All_Image/res/Maxmsize.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Maxmsize_Button.setIcon(icon1)
        self.Maxmsize_Button.setObjectName("Maxmsize_Button")
        self.Title_Layout.addWidget(self.Maxmsize_Button)
        self.Close_Button = QtWidgets.QPushButton(self.Title_Widget)
        self.Close_Button.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(":/All_Image/res/Close.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.Close_Button.setIcon(icon2)
        self.Close_Button.setObjectName("Close_Button")
        self.Title_Layout.addWidget(self.Close_Button)
        self.Central_Widget1.addWidget(self.Title_Widget)
        self.Central_Layout = QtWidgets.QHBoxLayout()
        self.Central_Layout.setContentsMargins(0, 0, -1, -1)
        self.Central_Layout.setSpacing(0)
        self.Central_Layout.setObjectName("Central_Layout")
        self.Tab_Widget = QtWidgets.QWidget(self.Central_Widget)
        self.Tab_Widget.setStyleSheet("#Tab_Widget{\n"
"    background: rgb(66,66,88);\n"
"}\n"
"\n"
"#Tab_Widget QPushButton {\n"
"    padding-left: 40px;\n"
"    border-top: 1px;\n"
"    border-style: solid;\n"
"    border-radius: 0px;\n"
"    border-color: rgb(125,125,155);\n"
"    margin: 0px;\n"
"\n"
"    width: 140px;\n"
"    height: 60px;\n"
"\n"
"    background: rgb(85,85,115);\n"
"    color: rgb(255,255,255);\n"
"    \n"
"    font-family: \'MicroSoft YaHei Light\';\n"
"    font-size: 20px;\n"
"    text-align: left\n"
"}\n"
"#Tab_Widget QPushButton:hover {\n"
"    background: rgb(105,105,125);\n"
"}\n"
"#Tab_Widget QPushButton:pressed {\n"
"    background: rgb(125,125,145);\n"
"}\n"
"\n"
"")
        self.Tab_Widget.setObjectName("Tab_Widget")
        self.Tab_Layout = QtWidgets.QVBoxLayout(self.Tab_Widget)
        self.Tab_Layout.setContentsMargins(0, 0, 0, 0)
        self.Tab_Layout.setSpacing(10)
        self.Tab_Layout.setObjectName("Tab_Layout")
        self.Sim_Setting_Button = QtWidgets.QPushButton(self.Tab_Widget)
        self.Sim_Setting_Button.setStyleSheet("#Sim_Setting_Button{\n"
"    border-top: 0px;\n"
"    border-bottom-left-radius: 20px;\n"
"    border-bottom-right-radius: 20px;\n"
"}")
        self.Sim_Setting_Button.setObjectName("Sim_Setting_Button")
        self.Tab_Layout.addWidget(self.Sim_Setting_Button)
        self.Evaluation_Widget = QtWidgets.QWidget(self.Tab_Widget)
        self.Evaluation_Widget.setObjectName("Evaluation_Widget")
        self.Evaluation_Widget1 = QtWidgets.QVBoxLayout(self.Evaluation_Widget)
        self.Evaluation_Widget1.setContentsMargins(0, 0, 0, 0)
        self.Evaluation_Widget1.setSpacing(0)
        self.Evaluation_Widget1.setObjectName("Evaluation_Widget1")
        self.Safety_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Safety_Assess_Button.setStyleSheet("#Safety_Assess_Button{\n"
"    border-top: 0px;\n"
"    border-top-left-radius: 20px;\n"
"    border-top-right-radius: 20px;\n"
"}")
        self.Safety_Assess_Button.setObjectName("Safety_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Safety_Assess_Button)
        self.Conflict_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Conflict_Assess_Button.setObjectName("Conflict_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Conflict_Assess_Button)
        self.Stability_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Stability_Assess_Button.setObjectName("Stability_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Stability_Assess_Button)
        self.Effectiveness_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Effectiveness_Assess_Button.setObjectName("Effectiveness_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Effectiveness_Assess_Button)
        self.Coprehensive_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Coprehensive_Assess_Button.setObjectName("Coprehensive_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Coprehensive_Assess_Button)
        self.Service_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Service_Assess_Button.setObjectName("Service_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Service_Assess_Button)
        self.Carbin_Assess_Button = QtWidgets.QPushButton(self.Evaluation_Widget)
        self.Carbin_Assess_Button.setStyleSheet("#Carbin_Assess_Button{\n"
"    border-bottom-left-radius: 20px;\n"
"    border-bottom-right-radius: 20px;\n"
"}")
        self.Carbin_Assess_Button.setObjectName("Carbin_Assess_Button")
        self.Evaluation_Widget1.addWidget(self.Carbin_Assess_Button)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.Evaluation_Widget1.addItem(spacerItem1)
        self.Tab_Layout.addWidget(self.Evaluation_Widget)
        self.Central_Layout.addWidget(self.Tab_Widget)
        self.Main_Stacked_Widget = QtWidgets.QStackedWidget(self.Central_Widget)
        self.Main_Stacked_Widget.setStyleSheet("#Main_Stacked_Widget QLabel{\n"
"    color: rgb(255,255,255);\n"
"    \n"
"    font-family: \'MicroSoft YaHei Light\';\n"
"    font-size: 20px;\n"
"}\n"
"\n"
"#Main_Stacked_Widget QSlider::handle:horizontal{\n"
"    border: 3px;\n"
"    border-style: solid ;\n"
"    border-radius: 12px;\n"
"    border-color: rgb(255,255,255);\n"
"    margin: -11px 0px -11px 0px;\n"
"\n"
"    width: 18px;\n"
"    height: 20px;\n"
"\n"
"    background: rgb(46, 121, 199);\n"
"}\n"
"#Main_Stacked_Widget QSlider::groove:horizontal{\n"
"    height: 2px;\n"
"    background : rgb(219,219,219);\n"
"}\n"
"#Main_Stacked_Widget QSlider::add-page:horizontal{\n"
"    background-color: rgb(219,219,219);\n"
"}\n"
"#Main_Stacked_Widget QSlider::sub-page:horizontal{\n"
"    background-color: rgb(16, 71, 169);\n"
"}\n"
"\n"
"#Main_Stacked_Widget QLineEdit{\n"
"    padding-left: 10px;\n"
"    border: 0px;\n"
"    border-radius: 8px;\n"
"\n"
"    width: 40px;\n"
"    height: 27px;\n"
"\n"
"    background: rgb(22, 22, 37);\n"
"    color: rgb(255,255,255);\n"
"    \n"
"    font-family: \'MicroSoft YaHei Light\';\n"
"}\n"
"\n"
"#Main_Stacked_Widget QPushButton {\n"
"    \n"
"    margin: 0px;\n"
"    border: 0px;\n"
"    border-radius: 20px;\n"
"\n"
"    width: 140px;\n"
"    height: 60px;\n"
"\n"
"    background: rgb(85,85,115);\n"
"    color: rgb(255,255,255);\n"
"    \n"
"    font-family: \'MicroSoft YaHei Light\';\n"
"    font-size: 20px;\n"
"    text-align: center\n"
"}\n"
"  ")
        self.Main_Stacked_Widget.setObjectName("Main_Stacked_Widget")
        self.Setting_Page = QtWidgets.QWidget()
        self.Setting_Page.setObjectName("Setting_Page")
        self.Setting_Page_2 = QtWidgets.QVBoxLayout(self.Setting_Page)
        self.Setting_Page_2.setObjectName("Setting_Page_2")
        self.Sim_Setting_Grid_Widget = QtWidgets.QWidget(self.Setting_Page)
        self.Sim_Setting_Grid_Widget.setObjectName("Sim_Setting_Grid_Widget")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.Sim_Setting_Grid_Widget)
        self.gridLayout_3.setHorizontalSpacing(20)
        self.gridLayout_3.setVerticalSpacing(6)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.Ramp_Traffic_Cell_Widget = QtWidgets.QWidget(self.Sim_Setting_Grid_Widget)
        self.Ramp_Traffic_Cell_Widget.setObjectName("Ramp_Traffic_Cell_Widget")
        self.Ramp_Traffic_Cell_Widget1 = QtWidgets.QHBoxLayout(self.Ramp_Traffic_Cell_Widget)
        self.Ramp_Traffic_Cell_Widget1.setSpacing(10)
        self.Ramp_Traffic_Cell_Widget1.setObjectName("Ramp_Traffic_Cell_Widget1")
        self.Ramp_Traffic_Label = QtWidgets.QLabel(self.Ramp_Traffic_Cell_Widget)
        self.Ramp_Traffic_Label.setObjectName("Ramp_Traffic_Label")
        self.Ramp_Traffic_Cell_Widget1.addWidget(self.Ramp_Traffic_Label)
        self.Ramp_Traffi_Slider = QtWidgets.QSlider(self.Ramp_Traffic_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Ramp_Traffi_Slider.sizePolicy().hasHeightForWidth())
        self.Ramp_Traffi_Slider.setSizePolicy(sizePolicy)
        self.Ramp_Traffi_Slider.setMinimumSize(QtCore.QSize(0, 24))
        self.Ramp_Traffi_Slider.setOrientation(QtCore.Qt.Horizontal)
        self.Ramp_Traffi_Slider.setObjectName("Ramp_Traffi_Slider")
        self.Ramp_Traffic_Cell_Widget1.addWidget(self.Ramp_Traffi_Slider)
        self.Ramp_Traffic_LineEdit = QtWidgets.QLineEdit(self.Ramp_Traffic_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Ramp_Traffic_LineEdit.sizePolicy().hasHeightForWidth())
        self.Ramp_Traffic_LineEdit.setSizePolicy(sizePolicy)
        self.Ramp_Traffic_LineEdit.setObjectName("Ramp_Traffic_LineEdit")
        self.Ramp_Traffic_Cell_Widget1.addWidget(self.Ramp_Traffic_LineEdit)
        self.Ramp_Traffic_Cell_Widget1.setStretch(0, 1)
        self.Ramp_Traffic_Cell_Widget1.setStretch(1, 1)
        self.Ramp_Traffic_Cell_Widget1.setStretch(2, 1)
        self.gridLayout_3.addWidget(self.Ramp_Traffic_Cell_Widget, 0, 1, 1, 1)
        self.Main_Traffic_Cell_Widget = QtWidgets.QWidget(self.Sim_Setting_Grid_Widget)
        self.Main_Traffic_Cell_Widget.setObjectName("Main_Traffic_Cell_Widget")
        self.Main_Traffic_Cell_Widget1 = QtWidgets.QHBoxLayout(self.Main_Traffic_Cell_Widget)
        self.Main_Traffic_Cell_Widget1.setObjectName("Main_Traffic_Cell_Widget1")
        self.Main_Traffic_Label = QtWidgets.QLabel(self.Main_Traffic_Cell_Widget)
        self.Main_Traffic_Label.setObjectName("Main_Traffic_Label")
        self.Main_Traffic_Cell_Widget1.addWidget(self.Main_Traffic_Label)
        self.Main_Traffic_Slider = QtWidgets.QSlider(self.Main_Traffic_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Main_Traffic_Slider.sizePolicy().hasHeightForWidth())
        self.Main_Traffic_Slider.setSizePolicy(sizePolicy)
        self.Main_Traffic_Slider.setMinimumSize(QtCore.QSize(0, 24))
        self.Main_Traffic_Slider.setOrientation(QtCore.Qt.Horizontal)
        self.Main_Traffic_Slider.setObjectName("Main_Traffic_Slider")
        self.Main_Traffic_Cell_Widget1.addWidget(self.Main_Traffic_Slider)
        self.Main_Traffic_LineEdit = QtWidgets.QLineEdit(self.Main_Traffic_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Main_Traffic_LineEdit.sizePolicy().hasHeightForWidth())
        self.Main_Traffic_LineEdit.setSizePolicy(sizePolicy)
        self.Main_Traffic_LineEdit.setObjectName("Main_Traffic_LineEdit")
        self.Main_Traffic_Cell_Widget1.addWidget(self.Main_Traffic_LineEdit)
        self.Main_Traffic_Cell_Widget1.setStretch(0, 1)
        self.Main_Traffic_Cell_Widget1.setStretch(1, 1)
        self.gridLayout_3.addWidget(self.Main_Traffic_Cell_Widget, 1, 1, 1, 1)
        self.Speed_Cell_Widget = QtWidgets.QWidget(self.Sim_Setting_Grid_Widget)
        self.Speed_Cell_Widget.setObjectName("Speed_Cell_Widget")
        self.Speed_Cell_Widget1 = QtWidgets.QHBoxLayout(self.Speed_Cell_Widget)
        self.Speed_Cell_Widget1.setObjectName("Speed_Cell_Widget1")
        self.Speed_Label = QtWidgets.QLabel(self.Speed_Cell_Widget)
        self.Speed_Label.setObjectName("Speed_Label")
        self.Speed_Cell_Widget1.addWidget(self.Speed_Label)
        self.Speed_Slider = QtWidgets.QSlider(self.Speed_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Speed_Slider.sizePolicy().hasHeightForWidth())
        self.Speed_Slider.setSizePolicy(sizePolicy)
        self.Speed_Slider.setMinimumSize(QtCore.QSize(0, 24))
        self.Speed_Slider.setOrientation(QtCore.Qt.Horizontal)
        self.Speed_Slider.setObjectName("Speed_Slider")
        self.Speed_Cell_Widget1.addWidget(self.Speed_Slider)
        self.Speed_LineEdit = QtWidgets.QLineEdit(self.Speed_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Speed_LineEdit.sizePolicy().hasHeightForWidth())
        self.Speed_LineEdit.setSizePolicy(sizePolicy)
        self.Speed_LineEdit.setObjectName("Speed_LineEdit")
        self.Speed_Cell_Widget1.addWidget(self.Speed_LineEdit)
        self.Speed_Cell_Widget1.setStretch(0, 1)
        self.Speed_Cell_Widget1.setStretch(1, 1)
        self.gridLayout_3.addWidget(self.Speed_Cell_Widget, 0, 2, 1, 1)
        self.Mixing_Rate_Cell_Widget = QtWidgets.QWidget(self.Sim_Setting_Grid_Widget)
        self.Mixing_Rate_Cell_Widget.setObjectName("Mixing_Rate_Cell_Widget")
        self.Mining_Rate_Cell_Widget = QtWidgets.QHBoxLayout(self.Mixing_Rate_Cell_Widget)
        self.Mining_Rate_Cell_Widget.setObjectName("Mining_Rate_Cell_Widget")
        self.Mixing_Rate_Label = QtWidgets.QLabel(self.Mixing_Rate_Cell_Widget)
        self.Mixing_Rate_Label.setObjectName("Mixing_Rate_Label")
        self.Mining_Rate_Cell_Widget.addWidget(self.Mixing_Rate_Label)
        self.Mixing_Rate_Slider = QtWidgets.QSlider(self.Mixing_Rate_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Mixing_Rate_Slider.sizePolicy().hasHeightForWidth())
        self.Mixing_Rate_Slider.setSizePolicy(sizePolicy)
        self.Mixing_Rate_Slider.setMinimumSize(QtCore.QSize(0, 24))
        self.Mixing_Rate_Slider.setOrientation(QtCore.Qt.Horizontal)
        self.Mixing_Rate_Slider.setObjectName("Mixing_Rate_Slider")
        self.Mining_Rate_Cell_Widget.addWidget(self.Mixing_Rate_Slider)
        self.Mixing_Rate_LineEdit = QtWidgets.QLineEdit(self.Mixing_Rate_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Mixing_Rate_LineEdit.sizePolicy().hasHeightForWidth())
        self.Mixing_Rate_LineEdit.setSizePolicy(sizePolicy)
        self.Mixing_Rate_LineEdit.setObjectName("Mixing_Rate_LineEdit")
        self.Mining_Rate_Cell_Widget.addWidget(self.Mixing_Rate_LineEdit)
        self.Mining_Rate_Cell_Widget.setStretch(0, 1)
        self.Mining_Rate_Cell_Widget.setStretch(1, 1)
        self.gridLayout_3.addWidget(self.Mixing_Rate_Cell_Widget, 1, 2, 1, 1)
        self.Intel_Ratio_Cell_Widget = QtWidgets.QWidget(self.Sim_Setting_Grid_Widget)
        self.Intel_Ratio_Cell_Widget.setObjectName("Intel_Ratio_Cell_Widget")
        self.Intel_Rate_Cell_Widget = QtWidgets.QHBoxLayout(self.Intel_Ratio_Cell_Widget)
        self.Intel_Rate_Cell_Widget.setObjectName("Intel_Rate_Cell_Widget")
        self.Intel_Rate_Label = QtWidgets.QLabel(self.Intel_Ratio_Cell_Widget)
        self.Intel_Rate_Label.setObjectName("Intel_Rate_Label")
        self.Intel_Rate_Cell_Widget.addWidget(self.Intel_Rate_Label)
        self.Intel_Rate_Slider = QtWidgets.QSlider(self.Intel_Ratio_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Intel_Rate_Slider.sizePolicy().hasHeightForWidth())
        self.Intel_Rate_Slider.setSizePolicy(sizePolicy)
        self.Intel_Rate_Slider.setMinimumSize(QtCore.QSize(0, 24))
        self.Intel_Rate_Slider.setOrientation(QtCore.Qt.Horizontal)
        self.Intel_Rate_Slider.setObjectName("Intel_Rate_Slider")
        self.Intel_Rate_Cell_Widget.addWidget(self.Intel_Rate_Slider)
        self.Intel_Rate_LineEdit = QtWidgets.QLineEdit(self.Intel_Ratio_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Intel_Rate_LineEdit.sizePolicy().hasHeightForWidth())
        self.Intel_Rate_LineEdit.setSizePolicy(sizePolicy)
        self.Intel_Rate_LineEdit.setObjectName("Intel_Rate_LineEdit")
        self.Intel_Rate_Cell_Widget.addWidget(self.Intel_Rate_LineEdit)
        self.Intel_Rate_Cell_Widget.setStretch(0, 1)
        self.Intel_Rate_Cell_Widget.setStretch(1, 1)
        self.gridLayout_3.addWidget(self.Intel_Ratio_Cell_Widget, 0, 3, 1, 1)
        self.Sim_Step_Cell_Widget = QtWidgets.QWidget(self.Sim_Setting_Grid_Widget)
        self.Sim_Step_Cell_Widget.setObjectName("Sim_Step_Cell_Widget")
        self.Sim_Step_Cell_Widget1 = QtWidgets.QHBoxLayout(self.Sim_Step_Cell_Widget)
        self.Sim_Step_Cell_Widget1.setObjectName("Sim_Step_Cell_Widget1")
        self.Sim_Step_Label = QtWidgets.QLabel(self.Sim_Step_Cell_Widget)
        self.Sim_Step_Label.setObjectName("Sim_Step_Label")
        self.Sim_Step_Cell_Widget1.addWidget(self.Sim_Step_Label)
        self.Sim_Step_Slider = QtWidgets.QSlider(self.Sim_Step_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Sim_Step_Slider.sizePolicy().hasHeightForWidth())
        self.Sim_Step_Slider.setSizePolicy(sizePolicy)
        self.Sim_Step_Slider.setMinimumSize(QtCore.QSize(0, 24))
        self.Sim_Step_Slider.setOrientation(QtCore.Qt.Horizontal)
        self.Sim_Step_Slider.setObjectName("Sim_Step_Slider")
        self.Sim_Step_Cell_Widget1.addWidget(self.Sim_Step_Slider)
        self.Sim_Step_LineEdit = QtWidgets.QLineEdit(self.Sim_Step_Cell_Widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Sim_Step_LineEdit.sizePolicy().hasHeightForWidth())
        self.Sim_Step_LineEdit.setSizePolicy(sizePolicy)
        self.Sim_Step_LineEdit.setObjectName("Sim_Step_LineEdit")
        self.Sim_Step_Cell_Widget1.addWidget(self.Sim_Step_LineEdit)
        self.Sim_Step_Cell_Widget1.setStretch(0, 1)
        self.Sim_Step_Cell_Widget1.setStretch(1, 1)
        self.gridLayout_3.addWidget(self.Sim_Step_Cell_Widget, 1, 3, 1, 1)
        self.Setting_Page_2.addWidget(self.Sim_Setting_Grid_Widget)
        self.Control_Widget = QtWidgets.QVBoxLayout()
        self.Control_Widget.setObjectName("Control_Widget")
        self.Info_Label = QtWidgets.QLabel(self.Setting_Page)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Info_Label.sizePolicy().hasHeightForWidth())
        self.Info_Label.setSizePolicy(sizePolicy)
        self.Info_Label.setStyleSheet("#Info_Label{\n"
"    padding-left: 40px;\n"
"}")
        self.Info_Label.setObjectName("Info_Label")
        self.Control_Widget.addWidget(self.Info_Label)
        self.Control_Panel_Widget = QtWidgets.QWidget(self.Setting_Page)
        self.Control_Panel_Widget.setObjectName("Control_Panel_Widget")
        self.Control_Panel_Widget_2 = QtWidgets.QHBoxLayout(self.Control_Panel_Widget)
        self.Control_Panel_Widget_2.setSpacing(20)
        self.Control_Panel_Widget_2.setObjectName("Control_Panel_Widget_2")
        self.Start_Sim_Button = QtWidgets.QPushButton(self.Control_Panel_Widget)
        self.Start_Sim_Button.setObjectName("Start_Sim_Button")
        self.Control_Panel_Widget_2.addWidget(self.Start_Sim_Button)
        self.End_Sim_Button = QtWidgets.QPushButton(self.Control_Panel_Widget)
        self.End_Sim_Button.setObjectName("End_Sim_Button")
        self.Control_Panel_Widget_2.addWidget(self.End_Sim_Button)
        self.Export_Data_Button = QtWidgets.QPushButton(self.Control_Panel_Widget)
        self.Export_Data_Button.setObjectName("Export_Data_Button")
        self.Control_Panel_Widget_2.addWidget(self.Export_Data_Button)
        self.Help_Button = QtWidgets.QPushButton(self.Control_Panel_Widget)
        self.Help_Button.setObjectName("Help_Button")
        self.Control_Panel_Widget_2.addWidget(self.Help_Button)
        self.Control_Widget.addWidget(self.Control_Panel_Widget)
        self.Setting_Page_2.addLayout(self.Control_Widget)
        self.Setting_Page_2.setStretch(0, 3)
        self.Setting_Page_2.setStretch(1, 1)
        self.Main_Stacked_Widget.addWidget(self.Setting_Page)
        self.Central_Layout.addWidget(self.Main_Stacked_Widget)
        self.Central_Widget1.addLayout(self.Central_Layout)
        Main_Window_UI.setCentralWidget(self.Central_Widget)

        self.retranslateUi(Main_Window_UI)
        self.Main_Stacked_Widget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Main_Window_UI)

    def retranslateUi(self, Main_Window_UI):
        _translate = QtCore.QCoreApplication.translate
        Main_Window_UI.setWindowTitle(_translate("Main_Window_UI", "MainWindow"))
        self.Sim_Setting_Button.setText(_translate("Main_Window_UI", "仿真设置"))
        self.Safety_Assess_Button.setText(_translate("Main_Window_UI", "安全评价"))
        self.Conflict_Assess_Button.setText(_translate("Main_Window_UI", "冲突评价"))
        self.Stability_Assess_Button.setText(_translate("Main_Window_UI", "稳定性评价"))
        self.Effectiveness_Assess_Button.setText(_translate("Main_Window_UI", "效率评价"))
        self.Coprehensive_Assess_Button.setText(_translate("Main_Window_UI", "综合评价"))
        self.Service_Assess_Button.setText(_translate("Main_Window_UI", "服务水平"))
        self.Carbin_Assess_Button.setText(_translate("Main_Window_UI", "碳排放评价"))
        self.Ramp_Traffic_Label.setText(_translate("Main_Window_UI", "匝道车流量"))
        self.Ramp_Traffic_LineEdit.setText(_translate("Main_Window_UI", "0"))
        self.Main_Traffic_Label.setText(_translate("Main_Window_UI", "主路车流量"))
        self.Main_Traffic_LineEdit.setText(_translate("Main_Window_UI", "0"))
        self.Speed_Label.setText(_translate("Main_Window_UI", "速度设定"))
        self.Speed_LineEdit.setText(_translate("Main_Window_UI", "0"))
        self.Mixing_Rate_Label.setText(_translate("Main_Window_UI", "大车混入率"))
        self.Mixing_Rate_LineEdit.setText(_translate("Main_Window_UI", "0"))
        self.Intel_Rate_Label.setText(_translate("Main_Window_UI", "智能车比例"))
        self.Intel_Rate_LineEdit.setText(_translate("Main_Window_UI", "0"))
        self.Sim_Step_Label.setText(_translate("Main_Window_UI", "仿真步长"))
        self.Sim_Step_LineEdit.setText(_translate("Main_Window_UI", "0"))
        self.Info_Label.setText(_translate("Main_Window_UI", "提示信息"))
        self.Start_Sim_Button.setText(_translate("Main_Window_UI", "开始仿真"))
        self.End_Sim_Button.setText(_translate("Main_Window_UI", "结束仿真"))
        self.Export_Data_Button.setText(_translate("Main_Window_UI", "导出数据"))
        self.Help_Button.setText(_translate("Main_Window_UI", "查看帮助"))
import Main_Window_UI_rc
