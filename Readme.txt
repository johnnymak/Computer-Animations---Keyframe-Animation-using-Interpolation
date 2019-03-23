
Johnny Mak (40002140)
Comp 477 - Computer Animations
Assignment #2


Input: 

- T: Switch between Editing and Animation Mode

  Editing Mode:
    - y : Add Keyframe
    - - : View Previous keyframe
    - + : View Next keyframe
    - 1 : Select Matrix Linear Interpolation
    - 2 : Select Euler Angle Linear Interpolation
    - 3 : Select Quaternion Linear Interpolation
    - 4 : Select Quaternion Spherical Linear Interpolation

    - s : Save configuration into file
    - l : Load configuration from file

  Animation Mode:
    - j : Decrease animation speed
    - k : Increase animation speed
    - p : Toggle between single and continuous playback



The calculations used for Quaternion, Matrix, and Euler Angle conversions are based off this website: 

	http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm