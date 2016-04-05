// \fn imgMatrix
// \brief Create matrix from image
function imgMatrix = loadImage(path,isRGB)
    if isRGB == 0 then
        imgMatrix = double(imread(path));
    else
        imgMatrix = double(rgb2gray(imread(path)));
    end 
endfunction

// vfn displayImage
// \brief Display an image from its matrix
function displayImage(imgMatrix)
    imshow(uint8(imgMatrix));
endfunction

// \fn writeImage
// \brief Write an image from its matrix
function image = writeImage(imgMatrix,fileName)
    image = imwrite(imgMatrix,fileName);
endfunction

// \fn applyMask
// \brief apply a given filter to an image
// \args imgMatrix: img to treat, mask: mask to use
// \return image: image which has been treated
//function matrixResult = applyMask(imgMatrix,mask)
function applyMask(imgMatrix,mask)
    // Normalize the mask
    mask = (1/sum(mask)).*mask;
    // Get size of the matrix
    [N, M] = size(imgMatrix); 
    // Get size of the mask 
    [NMask, MMask] = size(mask);
    // Get the halfSize of the mask provided to avoid problems in the edges of the pic
    halfSizeX = floor(NMask/2);      
    halfSizeY = floor(MMask/2);

    XsizeOfMatrixResult = N+2*halfSizeX
    YsizeOfMatrixResult = M+2*halfSizeY

    // Initialise the result matrix
    matrixResult=zeros(XsizeOfMatrixResult,YsizeOfMatrixResult);

    // Initialise temp bigger matrix for treatment
    tempMatrix=zeros(XsizeOfMatrixResult,YsizeOfMatrixResult);
    //tempMatrix=[1.1,1.2,1.3,1.4,1.5,1.6,1.7;2.1,2.2,2.3,2.4,2.5,2.6,2.7;3.1,3.2,3.3,3.4,3.5,3.6,3.7;4.1,4.2,4.3,4.4,4.5,4.6,4.7;5.1,5.2,5.3,5.4,5.5,5.6,5.7;6.1,6.2,6.3,6.4,6.5,6.6,6.7;7.1,7.2,7.3,7.4,7.5,7.6,7.7]
    disp(imgMatrix)
    disp(tempMatrix)
    [Ntemp, Mtemp] = size(tempMatrix)
    tempMatrix(halfSizeX+1:Ntemp-halfSizeX,halfSizeY+1:Mtemp-halfSizeY) = imgMatrix

    // Debug
    disp(tempMatrix)
    disp(imgMatrix(1,1))
    disp(tempMatrix(1,1)) // Should print 0 0
    disp(tempMatrix(halfSizeX+1,halfSizeY+1)) // Should print same as disp(imgMatrix(1,1)) 

    // treatment loop
    for i = halfSizeX:N
        for j = halfSizeY:M

        end
    end
endfunction
