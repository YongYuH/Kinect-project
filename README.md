# Kinect-project
- Final goal：Build 3D color human body model with Kinect
- Current function：
  1. Extract the human 3Dpoints with color information from the raw color and depth information 
    - (Automatic detect 8 stationary views).
  2. Get the camera coordinate information from the board on the ground. 
    - (In order to get the information to do the cylindrical coodinate transform)

## [Demo Video](https://www.youtube.com/watch?v=VvN5n3h6lOk)


## Software to view the output point cloud data (.asc and .txt)
- [CloudCompare](http://www.danielgm.net/cc/)

## Environment Setting
- Windows 8
- Visual Studio 2013
- MFC 4.5

## Hardware 
- Kinect for Xbox One (v2)

## Software needed for executing the program (In the ExternalLib folder)
- [Kinect SDK v2.0](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
- [OpenCV 3.0.0](http://opencv.org/downloads.html)

## Update Information
### 2015/12/15 
###### Able to automatic sense the human motion and output the 3D point cloud file from different 8 angles
- (.asc：x,y,z for each pixel), (.txt：x,y,z,r,g,b for each pixel), and (.bmp：human mask during the filter process)
- the output array index(for 3D points and color information) and the user wait time need to test.

### 2015/12/20
###### Able to get information of the board (coordinates of corners and the equation of the plane of the board)

### 2016/3/16
###### Add demo video and link of CloudCompare

## Capture process
1. Click the "Background" button to get the image of background.
2. Click the "Board" button to get the image and the 3D points of background with the board.
3. Click the "Coordinate" button to 
  - get the 2D pixels and 3D points of the corners on the board 
  - and get the equation of the plane of the board.
4. Click the "Capture" button to start the automatic human 3D points capture process, 
  - if the user walks in front of the Kinect v2 in certain area, 
  - the program would automatically detect the stationary state of the user, 
  - and starts to capture points of 8 angles, after the information of each angle has been successfully captured, 
  - the program would play a sound effect to guide the user to turn to another angle.
5. Click the "Output" button to start output the 3D human points (asc and txt file).
