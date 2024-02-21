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
        self.reflineButton = QtW.QPushButton("show/hide reference line", self.widget)
        self.reflineButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.reflineButton)
        self.trackboundsButton = QtW.QPushButton("show/hide trackbounds", self.widget)
        self.trackboundsButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.trackboundsButton)
        self.conesButton = QtW.QPushButton("show/hide cones", self.widget)
        self.conesButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.conesButton)
        self.mincurvButton = QtW.QPushButton("show/hide minimum curvature", self.widget)
        self.mincurvButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.mincurvButton)
        self.computeButton = QtW.QPushButton("compute minimum curvature", self.widget)
        self.computeButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.computeButton)

        # Connect the button's clicked signal to a slot
        self.reflineButton.clicked.connect(self.refline_clicked)
        self.trackboundsButton.clicked.connect(self.trackbounds_clicked)
        self.conesButton.clicked.connect(self.cones_clicked)
        self.mincurvButton.clicked.connect(self.mincurv_clicked)
        self.computeButton.clicked.connect(self.compute_clicked)

    def refline_clicked(self):
        self.widget.refline_on = not self.widget.refline_on
        if self.widget.refline_on:
            self.reflineButton.setStyleSheet("background-color: green")
        else:
            self.reflineButton.setStyleSheet("background-color: red")
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

    def compute_clicked(self):
        self.widget.compute_on = True
        self.computeButton.setStyleSheet("background-color: orange")

    def set_buttons(self):
        if self.widget.refline_on:
            self.reflineButton.setStyleSheet("background-color: green")
        else:
            self.reflineButton.setStyleSheet("background-color: gray")
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

        self.computeButton.setStyleSheet("background-color: gray")
