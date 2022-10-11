# Camera calibration
## How to use

 1. Print a chessboard or charuco pattern and lay it on a flat surface. Charuco boards are preferred since they are more robust. 
 2. Take about 15 pictures of that chessboard while changing the distance and the angle every time. Make sure the chessboard is clearly visible!
 3. Run `python3 camera_calibration {img} {size} {square_length} {marker_length} {out} {board_type}` where:
	 1. **{img}** is a mask for all the .jpg images
	 2. **{size}** is the grid size of the chessboard (wxh)
	 3. **{square_length}** is the physical length of a side of a square from the chessboard
	 4. **{marker_length}** is the physical length of a side of a marker inside a square. This is only relevant when using charuco boards. When using a chessboard pattern this value will be ignored. 
	 4. **{out}** is the .npz output file containing the distortion and camera calibration matrices. 
	 5. **{board_type}** is the type of board used to calibrate the camera. Can be "chess" or "charuco"

## Demo
### Chessboard
There are pictures of the Baumer camera using a chessboard pattern [on the SharePoint](https://ugentracing.sharepoint.com/:u:/s/UGR9/Eas-okN5GxBOtmj9hvf8AlABngDPR5ZwewN-C7AyzHxYzg?e=5Gijb7). To run the demo, run `python3 camera_calibration "folder_with_images/*.jpg" 6x9 2.216666 0 output.npz chess`

If you see something like this in the end (it may be on a different picture, but the point is that the axes should match the square like this) then it is ok.
![image](https://user-images.githubusercontent.com/22452540/127037049-d02955c4-097e-4e00-a0ec-4f4d57d4952e.png)

### Charuco board
There are demo pictures (these are at least a year old) of the Baumer camera using a charuco board pattern (type 5x5_50) in /camera_calibration/data/baumer_5x5_50 folder (again in the 2022_autonomous_binaries repository). To run the demo, run `python3 camera_calibration "{autonomous_binaries_repo}/camera_calibration/data/baumer_5x5_50/*.jpg" 8x10 35 26 output.npz charuco`
