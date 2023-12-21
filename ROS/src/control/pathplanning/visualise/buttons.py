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
        self.loopButton = QtW.QPushButton("close/unclose loop", self.widget)
        self.loopButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.loopButton)
        self.middellineButton = QtW.QPushButton("show/hide middelline", self.widget)
        self.middellineButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.middellineButton)
        self.trackboundsButton = QtW.QPushButton("show/hide trackbounds", self.widget)
        self.trackboundsButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.trackboundsButton)
        self.debugCenterPointsButton = QtW.QPushButton(
            "enable/disable debug center points", self.widget
        )
        self.debugCenterPointsButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.debugCenterPointsButton)
        self.debugBadPointsButton = QtW.QPushButton(
            "enable/disable debug bad points", self.widget
        )
        self.debugBadPointsButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.debugBadPointsButton)
        self.selectAllButton = QtW.QPushButton("select all cones", self.widget)
        self.selectAllButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.selectAllButton)
        self.deselectAllButton = QtW.QPushButton("deselect all cones", self.widget)
        self.deselectAllButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.deselectAllButton)
        self.placeConesButton = QtW.QPushButton(
            "enable/disable placing cones", self.widget
        )
        self.placeConesButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(self.placeConesButton)

        # Connect the button's clicked signal to a slot
        self.loopButton.clicked.connect(self.close_loop_clicked)
        self.middellineButton.clicked.connect(self.middelline_clicked)
        self.selectAllButton.clicked.connect(self.select_all_clicked)
        self.deselectAllButton.clicked.connect(self.deselect_all_clicked)
        self.trackboundsButton.clicked.connect(self.trackbounds_clicked)
        self.placeConesButton.clicked.connect(self.place_cones_clicked)
        self.debugCenterPointsButton.clicked.connect(self.debug_center_points_clicked)
        self.debugBadPointsButton.clicked.connect(self.debug_bad_points_clicked)

    def close_loop_clicked(self):
        self.widget.is_closed = not self.widget.is_closed
        if self.widget.is_closed:
            self.loopButton.setStyleSheet("background-color: green")
        else:
            self.loopButton.setStyleSheet("background-color: red")
        self.widget.update()

    def middelline_clicked(self):
        self.widget.middelline_on = not self.widget.middelline_on
        if self.widget.middelline_on:
            self.middellineButton.setStyleSheet("background-color: green")
        else:
            self.middellineButton.setStyleSheet("background-color: red")
        self.widget.update()

    def trackbounds_clicked(self):
        self.widget.trackbounds_on = not self.widget.trackbounds_on
        if self.widget.trackbounds_on:
            self.trackboundsButton.setStyleSheet("background-color: green")
        else:
            self.trackboundsButton.setStyleSheet("background-color: red")
        self.widget.update()

    def debug_center_points_clicked(self):
        self.widget.debug_centerPoints = not self.widget.debug_centerPoints
        if self.widget.debug_centerPoints:
            self.debugCenterPointsButton.setStyleSheet("background-color: green")
        else:
            self.debugCenterPointsButton.setStyleSheet("background-color: red")
        self.widget.update()

    def debug_bad_points_clicked(self):
        self.widget.debug_badPoints = not self.widget.debug_badPoints
        if self.widget.debug_badPoints:
            self.debugBadPointsButton.setStyleSheet("background-color: green")
        else:
            self.debugBadPointsButton.setStyleSheet("background-color: red")
        self.widget.update()

    def select_all_clicked(self):
        self.widget.selected_blue_cones = []
        self.widget.selected_yellow_cones = []
        for cone in self.widget.yellow_cones:
            self.widget.selected_yellow_cones.append(cone)
        for cone in self.widget.blue_cones:
            self.widget.selected_blue_cones.append(cone)
        self.widget.empty_pathplanning_input()
        self.widget.update()

    def deselect_all_clicked(self):
        self.widget.selected_blue_cones = []
        self.widget.selected_yellow_cones = []
        self.widget.empty_pathplanning_input()
        self.widget.update()

    def place_cones_clicked(self):
        self.widget.place_cones = not self.widget.place_cones
        if self.widget.place_cones:
            self.placeConesButton.setStyleSheet("background-color: green")
        else:
            self.placeConesButton.setStyleSheet("background-color: red")

    def set_buttons(self):
        if self.widget.is_closed:
            self.loopButton.setStyleSheet("background-color: green")
        else:
            self.loopButton.setStyleSheet("background-color: red")
        if self.widget.middelline_on:
            self.middellineButton.setStyleSheet("background-color: green")
        else:
            self.middellineButton.setStyleSheet("background-color: red")
        if self.widget.trackbounds_on:
            self.trackboundsButton.setStyleSheet("background-color: green")
        else:
            self.trackboundsButton.setStyleSheet("background-color: red")
        if self.widget.debug_centerPoints:
            self.debugCenterPointsButton.setStyleSheet("background-color: green")
        else:
            self.debugCenterPointsButton.setStyleSheet("background-color: red")
        if self.widget.debug_badPoints:
            self.debugBadPointsButton.setStyleSheet("background-color: green")
        else:
            self.debugBadPointsButton.setStyleSheet("background-color: red")
        if self.widget.place_cones:
            self.placeConesButton.setStyleSheet("background-color: green")
        else:
            self.placeConesButton.setStyleSheet("background-color: red")
