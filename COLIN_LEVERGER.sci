// \fn imgMatrix
// \brief Create matrix from image
function imgMatrix = loadImage(path,isRGB)
    if isRGB == 0 then
        imgMatrix = double(imread(path));
    else
        imgMatrix = double(rgb2gray(imread(path)));
    end 
endfunction

// \fn displayImage
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
// \args imgMatrix: image to treat, mask: mask to use
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

    XsizeOfMatrixResult = N+2*halfSizeX;
    YsizeOfMatrixResult = M+2*halfSizeY;

    // Initialise temp bigger matrix for treatment
    tempMatrix=zeros(XsizeOfMatrixResult,YsizeOfMatrixResult);
    [Ntemp, Mtemp] = size(tempMatrix);

    // Debug: before treatment
    disp(imgMatrix);
    disp(tempMatrix);

    tempMatrix(halfSizeX+1:Ntemp-halfSizeX,halfSizeY+1:Mtemp-halfSizeY) = imgMatrix;

    // Debug
    disp(tempMatrix);
    disp(imgMatrix(1,1));
    disp(tempMatrix(1,1)); // Should print 0 0
    disp(tempMatrix(halfSizeX+1,halfSizeY+1)); // Should print same as disp(imgMatrix(1,1)) 

    // Treatment loop
    for i = halfSizeX+1:N-halfSizeX
        for j = halfSizeY+1:M-halfSizeY
            x2 = i - halfSizeX - 1;
            y2 = j - halfSizeY - 1;
            for xMask = 1:NMask
                for yMask = 1:MMask
                    maskPixel = mask(xMask,yMask);
                    imgPixel = imgMatrix(x2 + xMask, y2 + yMask);
                end
            end
        end 
    end
endfunction
