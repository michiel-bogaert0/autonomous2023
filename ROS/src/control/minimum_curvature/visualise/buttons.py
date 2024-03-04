from PyQt5 import QtCore as QtC
from PyQt5 import QtWidgets as QtW

# from visualise_pathplanning import MapWidget


class Buttons:
    def __init__(self, widget):
        self.widget = widget

        # Create a QVBoxLayout to hold the button and the map
        layout = QtW.QVBoxLayout(widget)
        layout.setAlignment(QtC.Qt.AlignTop | QtC.Qt.AlignRight)
        # Set spacing between buttons
        layout.setSpacing(10)

        # Create a QPushButton and add it to the layout
        self.centerlineButton = QtW.QPushButton("show/hide center line", self.widget)
        self.centerlineButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.centerlineButton)
        self.trackboundsButton = QtW.QPushButton("show/hide trackbounds", self.widget)
        self.trackboundsButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.trackboundsButton)
        self.conesButton = QtW.QPushButton("show/hide cones", self.widget)
        self.conesButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.conesButton)
        self.mincurvButton = QtW.QPushButton("show/hide minimum curvature", self.widget)
        self.mincurvButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.mincurvButton)
        self.extraButton = QtW.QPushButton("show/hide extra", self.widget)
        self.extraButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.extraButton)
        self.iqpButton = QtW.QPushButton("show/hide iqp", self.widget)
        self.iqpButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.iqpButton)
        self.reflineButton = QtW.QPushButton("show/hide reference", self.widget)
        self.reflineButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.reflineButton)

        self.computeButton = QtW.QPushButton("compute minimum curvature", self.widget)
        self.computeButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.computeButton)

        # Connect the button's clicked signal to a slot
        self.centerlineButton.clicked.connect(self.centerline_clicked)
        self.trackboundsButton.clicked.connect(self.trackbounds_clicked)
        self.conesButton.clicked.connect(self.cones_clicked)
        self.mincurvButton.clicked.connect(self.mincurv_clicked)
        self.extraButton.clicked.connect(self.extra_clicked)
        self.iqpButton.clicked.connect(self.iqp_clicked)
        self.reflineButton.clicked.connect(self.refline_clicked)
        self.computeButton.clicked.connect(self.compute_clicked)

    def centerline_clicked(self):
        self.widget.centerline_on = not self.widget.centerline_on
        if self.widget.centerline_on:
            self.centerlineButton.setStyleSheet("background-color: green")
        else:
            self.centerlineButton.setStyleSheet("background-color: red")
        self.widget.update()

    def trackbounds_clicked(self):
        self.widget.trackbounds_on = not self.widget.trackbounds_on
        if self.widget.trackbounds_on:
            self.trackboundsButton.setStyleSheet("background-color: green")
        else:
            self.trackboundsButton.setStyleSheet("background-color: red")
        self.widget.update()

    def cones_clicked(self):
        self.widget.cones_on = not self.widget.cones_on

        self.widget.selected_blue_cones = []
        self.widget.selected_yellow_cones = []
        if self.widget.cones_on:
            self.conesButton.setStyleSheet("background-color: green")
            for cone in self.widget.blue_cones:
                self.widget.selected_blue_cones.append(cone)
            for cone in self.widget.yellow_cones:
                self.widget.selected_yellow_cones.append(cone)
        else:
            self.conesButton.setStyleSheet("background-color: red")
        self.widget.update()

    def mincurv_clicked(self):
        self.widget.mincurv_on = not self.widget.mincurv_on
        if self.widget.mincurv_on:
            self.mincurvButton.setStyleSheet("background-color: green")
        else:
            self.mincurvButton.setStyleSheet("background-color: red")
        self.widget.update()

    def extra_clicked(self):
        self.widget.extra_on = not self.widget.extra_on
        if self.widget.extra_on:
            self.extraButton.setStyleSheet("background-color: green")
        else:
            self.extraButton.setStyleSheet("background-color: red")
        self.widget.update()

    def iqp_clicked(self):
        self.widget.iqp_on = not self.widget.iqp_on
        if self.widget.iqp_on:
            self.iqpButton.setStyleSheet("background-color: green")
        else:
            self.iqpButton.setStyleSheet("background-color: red")
        self.widget.update()

    def refline_clicked(self):
        self.widget.refline_on = not self.widget.refline_on
        if self.widget.refline_on:
            self.reflineButton.setStyleSheet("background-color: green")
        else:
            self.reflineButton.setStyleSheet("background-color: gray")
        self.widget.update()

    def compute_clicked(self):
        self.widget.compute_on = True
        self.computeButton.setStyleSheet("background-color: orange")

    def set_buttons(self):
        if self.widget.centerline_on:
            self.centerlineButton.setStyleSheet("background-color: green")
        else:
            self.centerlineButton.setStyleSheet("background-color: gray")

        if self.widget.trackbounds_on:
            self.trackboundsButton.setStyleSheet("background-color: green")
        else:
            self.trackboundsButton.setStyleSheet("background-color: red")

        if self.widget.cones_on:
            self.conesButton.setStyleSheet("background-color: green")
        else:
            self.conesButton.setStyleSheet("background-color: red")

        if self.widget.mincurv_on:
            self.mincurvButton.setStyleSheet("background-color: green")
        else:
            self.mincurvButton.setStyleSheet("background-color: red")

        if self.widget.extra_on:
            self.extraButton.setStyleSheet("background-color: green")
        else:
            self.extraButton.setStyleSheet("background-color: red")

        if self.widget.iqp_on:
            self.iqpButton.setStyleSheet("background-color: green")
        else:
            self.iqpButton.setStyleSheet("background-color: red")

        if self.widget.refline_on:
            self.reflineButton.setStyleSheet("background-color: green")
        else:
            self.reflineButton.setStyleSheet("background-color: gray")

        self.computeButton.setStyleSheet("background-color: gray")
