from sys import argv as SYS_argv
from sys import exit as SYS_exit
from Control_Panel_UI import Ui_Control_Panel_UI
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtGui import QIntValidator

EVALUATION_AREA_HEIGHT = 1000


class Buile_In_Evevt:
    def __init__(self, main_window):
        self.main_window = main_window

        self.UI_loaded = False

    def resizeEvent(self, event):
        if self.buile_in_evevt.UI_loaded:
            self.main_window.Evaluation_scrollArea_Widget.setGeometry(
                0, 0,
                self.main_window.Evaluation_scrollArea.geometry().width() - 26,
                EVALUATION_AREA_HEIGHT)
            self.main_window.Evaluatio_GridWidget.setGeometry(
                0, 0,
                self.main_window.Evaluation_scrollArea_Widget.geometry().width(
                ) - 26,
                self.main_window.Evaluation_scrollArea_Widget.geometry().
                height())
        else:
            self.buile_in_evevt.UI_loaded = True


class Setting_Panel(QMainWindow):
    def __init__(self, main_window):
        self.main_window = main_window

        self.main_lane_triffic = 0
        self.ramp_lane_triffic = 0
        self.truck_mixing_rate = 0
        self.speed_setting = 0
        self.auto_driving_rate = 0
        self.simulation_stride = 1

        self.current_model = 0

        QMainWindow.__init__(self)
        intValidator = QIntValidator(self)
        intValidator.setRange(0, 100)
        self.main_window.Main_Lane_Triffic_LineEdit.setValidator(intValidator)
        self.main_window.Ramp_Lane_Triffic_LineEdit.setValidator(intValidator)
        self.main_window.Truck_Mixing_Rate_LineEdit.setValidator(intValidator)
        self.main_window.Auto_Driving_Rate_LineEdit.setValidator(intValidator)
        self.main_window.Simulation_Stride_LineEdit.setValidator(intValidator)

        intValidator.setRange(0, 300)
        self.main_window.Speed_Setting_LineEdit.setValidator(intValidator)

        self.main_window.Main_Lane_Triffic_LineEdit.textChanged.connect(
            self.On_main_lane_triffic_lineedit_textChanged)
        self.main_window.Ramp_Lane_Triffic_LineEdit.textChanged.connect(
            self.On_ramp_lane_triffic_lineedit_textChanged)
        self.main_window.Truck_Mixing_Rate_LineEdit.textChanged.connect(
            self.On_truck_mixing_rate_lineedit_textChanged)
        self.main_window.Speed_Setting_LineEdit.textChanged.connect(
            self.On_speed_setting_lineedit_textChanged)
        self.main_window.Auto_Driving_Rate_LineEdit.textChanged.connect(
            self.On_auto_driving_rate_lineedit_textChanged)
        self.main_window.Simulation_Stride_LineEdit.textChanged.connect(
            self.On_simulation_stride_lineedit_textChanged)

        self.main_window.Main_Lane_Triffic_Scrollbar.valueChanged.connect(
            self.On_main_lane_triffic_scrollbar_valueChanged)
        self.main_window.Ramp_Lane_Triffic_Scrollbar.valueChanged.connect(
            self.On_ramp_lane_triffic_scrollbar_valueChanged)
        self.main_window.Truck_Mixing_Rate_Scrollbar.valueChanged.connect(
            self.On_truck_mixing_rate_scrollbar_valueChanged)
        self.main_window.Speed_Setting_Scrollbar.valueChanged.connect(
            self.On_speed_setting_scrollbar_valueChanged)
        self.main_window.Auto_Driving_Rate_Scrollbar.valueChanged.connect(
            self.On_auto_driving_rate_scrollbar_valueChanged)
        self.main_window.Simulation_Stride_Scrollbar.valueChanged.connect(
            self.On_simulation_stride_scrollbar_valueChanged)

        self.main_window.Following_Model_RadioB.toggled.connect(
            self.Running_following_model)
        self.main_window.Lane_Change_Model_RadioB.toggled.connect(
            self.Running_lane_change_model)

        self.main_window.New_Simulation_Button.clicked.connect(
            self.On_new_simulation_button_clicked)
        self.main_window.Pause_Simulation_Button.clicked.connect(
            self.On_pause_simulation_button_clicked)
        self.main_window.End_Simulation_Button.clicked.connect(
            self.On_end_simulation_button_clicked)
        self.main_window.Export_Data_Button.clicked.connect(
            self.On_exoprt_simulation_button_clicked)

    def On_main_lane_triffic_lineedit_textChanged(self):
        text = self.main_window.Main_Lane_Triffic_LineEdit.text()

        if len(text) == 0:
            text = '0'
            self.main_window.Main_Lane_Triffic_LineEdit.setText(text)
        elif text[0] == '0' and len(text) > 1:
            for i in range(len(text)):
                if text[0] == '0':
                    text = text.replace('0', '', 1)

            if len(text) == 0:
                text = '0'

        self.main_window.Main_Lane_Triffic_Scrollbar.setValue(eval(text))
        self.setting_panel.main_lane_triffic = eval(text)

    def On_ramp_lane_triffic_lineedit_textChanged(self):
        text = self.main_window.Ramp_Lane_Triffic_LineEdit.text()

        if len(text) == 0:
            text = '0'
            self.main_window.Ramp_Lane_Triffic_LineEdit.setText(text)
        elif text[0] == '0' and len(text) > 1:
            for i in range(len(text)):
                if text[0] == '0':
                    text = text.replace('0', '', 1)

            if len(text) == 0:
                text = '0'

        self.main_window.Ramp_Lane_Triffic_Scrollbar.setValue(eval(text))
        self.setting_panel.ramp_lane_triffic = eval(text)

    def On_truck_mixing_rate_lineedit_textChanged(self):
        text = self.main_window.Truck_Mixing_Rate_LineEdit.text()

        if len(text) == 0:
            text = '0'
            self.main_window.Truck_Mixing_Rate_LineEdit.setText(text)
        elif text[0] == '0' and len(text) > 1:
            for i in range(len(text)):
                if text[0] == '0':
                    text = text.replace('0', '', 1)

            if len(text) == 0:
                text = '0'

        self.main_window.Truck_Mixing_Rate_Scrollbar.setValue(eval(text))
        self.setting_panel.truck_mixing_rate = eval(text)

    def On_speed_setting_lineedit_textChanged(self):
        text = self.main_window.Speed_Setting_LineEdit.text()

        if len(text) == 0:
            text = '0'
            self.main_window.Speed_Setting_LineEdit.setText(text)
        elif text[0] == '0' and len(text) > 1:
            for i in range(len(text)):
                if text[0] == '0':
                    text = text.replace('0', '', 1)

            if len(text) == 0:
                text = '0'

        self.main_window.Speed_Setting_Scrollbar.setValue(eval(text))
        self.setting_panel.speed_setting = eval(text)

    def On_auto_driving_rate_lineedit_textChanged(self):
        text = self.main_window.Auto_Driving_Rate_LineEdit.text()

        if len(text) == 0:
            text = '0'
            self.main_window.Auto_Driving_Rate_LineEdit.setText(text)
        elif text[0] == '0' and len(text) > 1:
            for i in range(len(text)):
                if text[0] == '0':
                    text = text.replace('0', '', 1)

            if len(text) == 0:
                text = '0'

        self.main_window.Auto_Driving_Rate_Scrollbar.setValue(eval(text))
        self.setting_panel.auto_driving_rate = eval(text)

    def On_simulation_stride_lineedit_textChanged(self):
        text = self.main_window.Simulation_Stride_LineEdit.text()

        if len(text) == 0:
            text = '1'
            self.main_window.Simulation_Stride_LineEdit.setText(text)
        elif text[0] == '0' and len(text) > 1:
            for i in range(len(text)):
                if text[0] == '0':
                    text = text.replace('0', '', 1)

            if len(text) == 0:
                text = '1'

        self.main_window.Simulation_Stride_Scrollbar.setValue(eval(text))
        self.setting_panel.simulation_stride = eval(text)

    def On_main_lane_triffic_scrollbar_valueChanged(self):
        self.main_window.Main_Lane_Triffic_LineEdit.setText(
            str(self.main_window.Main_Lane_Triffic_Scrollbar.value()))
        pass

    def On_ramp_lane_triffic_scrollbar_valueChanged(self):
        self.main_window.Ramp_Lane_Triffic_LineEdit.setText(
            str(self.main_window.Ramp_Lane_Triffic_Scrollbar.value()))
        pass

    def On_truck_mixing_rate_scrollbar_valueChanged(self):
        self.main_window.Truck_Mixing_Rate_LineEdit.setText(
            str(self.main_window.Truck_Mixing_Rate_Scrollbar.value()))
        pass

    def On_speed_setting_scrollbar_valueChanged(self):
        self.main_window.Speed_Setting_LineEdit.setText(
            str(self.main_window.Speed_Setting_Scrollbar.value()))
        pass

    def On_auto_driving_rate_scrollbar_valueChanged(self):
        self.main_window.Auto_Driving_Rate_LineEdit.setText(
            str(self.main_window.Auto_Driving_Rate_Scrollbar.value()))
        pass

    def On_simulation_stride_scrollbar_valueChanged(self):
        self.main_window.Simulation_Stride_LineEdit.setText(
            str(self.main_window.Simulation_Stride_Scrollbar.value()))
        pass

    def Running_following_model(self, event):
        if event == True:
            self.current_model = 0

    def Running_lane_change_model(self, event):
        if event == True:
            self.current_model = 1

    def On_new_simulation_button_clicked(self):
        print(1)

    def On_pause_simulation_button_clicked(self):
        print(2)

    def On_end_simulation_button_clicked(self):
        print(3)

    def On_exoprt_simulation_button_clicked(self):
        print(4)


class Main_Window(QMainWindow, Ui_Control_Panel_UI):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)

        self.main_window = self

        self.buile_in_evevt = Buile_In_Evevt(self.main_window)
        self.setting_panel = Setting_Panel(self.main_window)

        self.main_window.Set_buile_in_evevt()
        self.main_window.Set_setting_panel()

        self.resizeEvent = self.buile_in_evevt.resizeEvent

    def Set_buile_in_evevt(self):
        self.buile_in_evevt.buile_in_evevt = self.buile_in_evevt
        self.buile_in_evevt.setting_panel = self.setting_panel

    def Set_setting_panel(self):
        self.setting_panel.buile_in_evevt = self.buile_in_evevt
        self.setting_panel.setting_panel = self.setting_panel


if __name__ == '__main__':
    app = QApplication(SYS_argv)
    main_window = Main_Window()
    main_window.show()
    SYS_exit(app.exec_())