# requirements

"numpy==1.21.0",
"pyyaml==5.4.1",
"click>=8.0.4",
"PyQt5==5.15.9"

# Run
  
  ```bash
  roslaunch ugr_launch visualise.launch layout:=<layout_name>.json
  ```
layout_name is the name of the layout in the layouts folder

# <ins>Manual

Draw a parcours by hand.

Select cones (with alt + click).

Check the path planned by pathplanningsalgorithm Triangulation, RRT or Mid.

Move the startposition.

The green circle is the start position of the car with the red dot pointing the start orientation

## Usage

**left-click** = yellow cone

**right-click** = blue cone

**alt+click** on cone = select/deselect cone for pathplaningalgorithm

**alt+click** = orange cone

**shift+click** = remove cone

**double click on a cone**  = insert a cone with that index

**click+drag** = drag cone, car or car orientation

**ctrl+drag** = drag map

**intuitive zoom**

**ctrl+S** = save in layouts folder

**bestaande track editten** a.d.h.v. de meegegeven kegels is mogelijk:
plaats eerst de track in de layouts map
launch de node met `layouts:=track_file_name.json`

## Input/Output format

```json
 {
  "parameters": {
    "is_closed": "closed track or not",
    "startpos_x": "x coordinate of startposition",
    "startpos_y": "y coordinate of startposition",
    "startrot": "orientation in radians of startposition"
  },
  "middle_line": [
    {
      "C1": [
        "x[m]",
        "y[m]"
      ],
      "M": [
        "x[m]",
        "y[m]"
      ],
      "C2": [
        "x[m]",
        "y[m]"
      ]
    },
    {}
  ],
  "cones": {
    "yellow": [
      {
        "pos": [
          "x[m]",
          "y[m]"
        ]
      },
      {}
    ],
    "blue": [
      {}
    ],
    "orange": [
      {}
    ]
  }
}
```