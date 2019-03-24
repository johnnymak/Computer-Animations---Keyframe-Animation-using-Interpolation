
Johnny Mak
Comp 477 - Computer Animations
Assignment #2


This assignment is based off the result of the first assignment. Using the model given in the first assignment, we must implement a system allowing the user to create an animation using keyframes. 
The keyframes are interpolated in four different methods: regular transofmration matrices, Euler angles, Quaternions (Linear Interpolation), and Quaternions (Spherical Linear Interpolation).

You may view the intructions PDF included for more details. 

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
