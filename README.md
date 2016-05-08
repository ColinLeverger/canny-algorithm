# Canny's algorithm: detect edges on a picture

Scilab implementation of the famous Canny's algorithm. 

The purpose of this project is to treat a given image to find its edges, and to create a new image to display them.

## Steps

1. Normalize given image: application of a Gaussian filter.
2. Compute gradient and gradient norm (E_s_,E_o_).
3. Delete non-maximum values from the image.
3. Hysteresis thresholding.
4. Display result (concat given image and result image).

Note that the hysteresis threshold can be changed: for a good value, pick something between 70% and 85%.

## Main functions

1. `applyMask(imgMatrix,mask)` : apply a given mask to a given image. Mask can look like `[1,2,1;2,4,2;1,2,1]`.
2. `hysteresisThreshold(image,Eo,Es,95);` : compute hysteresis threshold for the given image, last parameter is the threshold you want to pick.

## Warning

- Possible issues if you use Scilab x64 (Scilab x32 preferred)
- Don't forget to increase the stack size! `stack size('max');`

## How to use this code?

- Clone project.
- Run script
- Done!