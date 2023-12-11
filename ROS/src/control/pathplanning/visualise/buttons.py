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
        loopButton = QtW.QPushButton("close/unclose loop", self.widget)
        loopButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(loopButton)
        middellineButton = QtW.QPushButton("show/hide middelline", self.widget)
        middellineButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(middellineButton)
        trackboundsButton = QtW.QPushButton("show/hide trackbounds", self.widget)
        trackboundsButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(trackboundsButton)
        debugCenterPoints = QtW.QPushButton(
            "enable/disable debug center points", self.widget
        )
        debugCenterPoints.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(debugCenterPoints)
        debugBadPoints = QtW.QPushButton("enable/disable debug bad points", self.widget)
        debugBadPoints.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(debugBadPoints)
        selectAllButton = QtW.QPushButton("select all cones", self.widget)
        selectAllButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(selectAllButton)
        deselectAllButton = QtW.QPushButton("deselect all cones", self.widget)
        deselectAllButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(deselectAllButton)
        placeConesButton = QtW.QPushButton("enable/disable placing cones", self.widget)
        placeConesButton.setFixedSize(250, 30)  # Set the size of the button
        layout.addWidget(
            placeConesButton
        )  # Align the button to the right and top of the layout

        # Connect the button's clicked signal to a slot
        loopButton.clicked.connect(self.close_loop_clicked)
        middellineButton.clicked.connect(self.middelline_clicked)
        selectAllButton.clicked.connect(self.select_all_clicked)
        deselectAllButton.clicked.connect(self.deselect_all_clicked)
        trackboundsButton.clicked.connect(self.trackbounds_clicked)
        placeConesButton.clicked.connect(self.place_cones_clicked)
        debugCenterPoints.clicked.connect(self.debug_center_points_clicked)
        debugBadPoints.clicked.connect(self.debug_bad_points_clicked)

    def close_loop_clicked(self):
        self.widget.is_closed = not self.widget.is_closed
        self.widget.update()

    def middelline_clicked(self):
        self.widget.middelline_on = not self.widget.middelline_on
        self.widget.update()

    def trackbounds_clicked(self):
        self.widget.trackbounds_on = not self.widget.trackbounds_on
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

    def debug_center_points_clicked(self):
        self.widget.debug_centerPoints = not self.widget.debug_centerPoints
        self.widget.update()

    def debug_bad_points_clicked(self):
        self.widget.debug_badPoints = not self.widget.debug_badPoints
        self.widget.update()
