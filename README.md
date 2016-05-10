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

- Possible issues if you use Scilab x64 (Scilab x32 preferred).
- Don't forget to increase the stack size! `stack size('max');`

## How to use this code?

- Clone project.
- Run script.
    + `img = loadImage('./img3.jpg',1);` l.372: Choose image to load.
    + `mask = [2,4,5,4,2;4,9,12,9,4;5,12,15,12,5;4,9,12,9,4;2,4,5,4,2];` l.375: init your mask.
    + `writeImage(uint8(resultImage),'./tests/img3_res_95.jpg');` l.395: write result of execution on disk.
- Done!
- Tests & results of previous executions provided in "./tests" folder.

## Results & execution

Results of several execution can be found in "./tests" folder.

![Result for image1, thresold 86%](./tests/img1_res_86.jpg)

