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
        loopButton.setFixedSize(150, 30)  # Set the size of the button
        layout.addWidget(
            loopButton
        )  # Align the button to the right and top of the layout
        # Create a QPushButton and add it to the layout
        middellineButton = QtW.QPushButton("show/hide middelline", self.widget)
        middellineButton.setFixedSize(150, 30)  # Set the size of the button
        layout.addWidget(
            middellineButton
        )  # Align the button to the right and top of the layout
        # Create a QPushButton and add it to the layout
        selectAllButton = QtW.QPushButton("select all cones", self.widget)
        selectAllButton.setFixedSize(150, 30)  # Set the size of the button
        layout.addWidget(
            selectAllButton
        )  # Align the button to the right and top of the layout
        # Create a QPushButton and add it to the layout
        deselectAllButton = QtW.QPushButton("deselect all cones", self.widget)
        deselectAllButton.setFixedSize(150, 30)  # Set the size of the button
        layout.addWidget(
            deselectAllButton
        )  # Align the button to the right and top of the layout
        # Create a QPushbutton and add it to the layout
        trackboundsButton = QtW.QPushButton("show/hide trackbounds", self.widget)
        trackboundsButton.setFixedSize(150, 30)  # Set the size of the button
        layout.addWidget(
            trackboundsButton
        )  # Align the button to the right and top of the layout
        # Create a QPushbutton and add it to the layout
        placeConesButton = QtW.QPushButton("enable/disable placing cones", self.widget)
        placeConesButton.setFixedSize(150, 30)  # Set the size of the button
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

    def close_loop_clicked(self):
        # This method will be called when the button is clicked
        self.widget.is_closed = not self.widget.is_closed
        self.widget.update()

    def middelline_clicked(self):
        # This method will be called when the button is clicked
        self.widget.middelline_on = not self.widget.middelline_on
        self.widget.update()

    def trackbounds_clicked(self):
        # This method will be called when the button is clicked
        self.widget.trackbounds_on = not self.widget.trackbounds_on
        self.widget.update()

    def select_all_clicked(self):
        # This method will be called when the button is clicked
        self.widget.selected_blue_cones = []
        self.widget.selected_yellow_cones = []
        for cone in self.widget.yellow_cones:
            self.widget.selected_yellow_cones.append(cone)
        for cone in self.widget.blue_cones:
            self.widget.selected_blue_cones.append(cone)
        self.widget.empty_path()
        self.widget.update()

    def deselect_all_clicked(self):
        # This method will be called when the button is clicked
        self.widget.selected_blue_cones = []
        self.widget.selected_yellow_cones = []
        self.widget.empty_path()
        self.widget.update()

    def place_cones_clicked(self):
        # This method will be called when the button is clicked
        self.widget.place_cones = not self.widget.place_cones
