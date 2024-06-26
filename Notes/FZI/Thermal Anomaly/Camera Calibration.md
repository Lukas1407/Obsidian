> [!abstract] 
>  The process of estimating the parameters of a camera is called camera calibration

## Camera Parameters
### Internal Parameters
- E.g. focal length, optical center, and radial distortion coefficients of the lens.
### External Parameters
- This refers to the orientation (rotation and translation) of the camera with respect to some world coordinate system.

## Image coordinates to World Coordinates
The equation that relates a 3D point $(X_{w},Y_{w},Z_{w})$ from the real world to its projection $(u,v)$ in image coordinates is the following: 
![[quicklatex 1.webp#invert|200]]
- Where $P$ is a 3x4 Projection matrix consisting of 2 parts:
- the intrinsic matrix ($K$) that contains the intrinsic parameters and the extrinsic matrix ($\mathbf{R} \mid  \mathbf{t}$) that is combination of 3×3 rotation matrix $\mathbf{R}$ and a 3×1 translation $\mathbf{t}$ vector.
![[quicklatex 2.webp#invert|300]]
- he intrinsic matrix $K$ is upper triangular
![[quicklatex 3.webp#invert|200]] where, $f_x$, $f_y$ are the $x$ and $y$ focal lengths ( yes, they are usually the same ). $c_x, c_y$ are the $x$ and $y$ coordinates of the optical center in the image plane. Using the center of the image is usually a good enough approximation. $\gamma$ is the skew between the axes. It is usually 0.

## Intrinsic Matrix K
### Focal Length
- **Focal Lengths (( f_x, f_y ))**: These represent the focal length of the camera along the x and y axes. They are typically equal, indicating square pixels, and affect the scale of the image.
- **Optical Center (( c_x, c_y ))**: These are the coordinates of the optical center on the image plane, often approximated by the image center.
- **Skew Coefficient (( \gamma ))**: This measures the skewness between the x and y axes. It’s usually zero, meaning the axes are perpendicular.


## Camera Calibration Step by Step
![[camera-calibration-flowchart-1024x910.webp#invert|400]]
