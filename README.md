# Kinect-project
Goal：Build 3D color human body model with Kinect

# Environment Setting
- Visual Studio 2013
- MFC 4.5

# Hardware 
- Kinect for Xbox One (v2)

# Update Information
## 2015/12/15 
###### Able to automatic sense the human motion and output the 3D point cloud file from different 8 angles
- (.asc：x,y,z for each pixel), (.txt：x,y,z,r,g,b for each pixel), and (.bmp：human mask during the filter process)
- the output array index(for 3D points and color information) and the user wait time need to test.

# Capture process
1. Click the "Background" button to get the image of background.
2. Click the "Capture" button to start the automatic human 3D points capture process, if the user walks in front of the Kinect v2 in certain area, the program would automatically detect the stationary state of the user, and starts to capture points of 8 angles, after the information of each angle has been successfully captured, the program would play a sound effect to guide the user to turn to another angle.
3. Click the "Output" button to start output the 3D human points (asc and txt file).
