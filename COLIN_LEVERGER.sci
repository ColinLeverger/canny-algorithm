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
// \brief Apply a given filter to an image
// \args imgMatrix: image to treat, mask: mask to use
// \return matrixResult: image which has been treated
function matrixResult = applyMask(imgMatrix,mask)
    // Normalize the mask
    mask = (1/sum(mask)).*mask;
    // Get size of the matrix
    [N, M] = size(imgMatrix); 
    // Get size of the mask 
    [NMask, MMask] = size(mask);
    // Get the halfSize of the mask provided to avoid problems in the edges of the pic
    halfSizeX = floor(NMask/2);      
    halfSizeY = floor(MMask/2);

    XsizeOfTempMatrix = N+2*halfSizeX;
    YsizeOfTempMatrix = M+2*halfSizeY;

    // Initialise temp bigger matrix for treatment
    tempMatrix=zeros(XsizeOfTempMatrix,YsizeOfTempMatrix);
    [Ntemp, Mtemp] = size(tempMatrix);

    // Debug: before treatment
    //disp(imgMatrix);
    //disp(tempMatrix);

    tempMatrix(halfSizeX+1:Ntemp-halfSizeX,halfSizeY+1:Mtemp-halfSizeY) = imgMatrix;

    // Debug
    //disp(tempMatrix);
    //disp(imgMatrix(1,1));
    //disp(tempMatrix(1,1)); // Should print 0 0
    //disp(tempMatrix(halfSizeX+1,halfSizeY+1)); // Should print same as disp(imgMatrix(1,1)) 

    // Init matrixResult
    matrixResult=zeros(N,M)

    // Treatment loop
    for i = halfSizeX + 1 : N + halfSizeX
        for j = halfSizeY + 1 : M + halfSizeY
            x2 = i - halfSizeX - 1;
            y2 = j - halfSizeY - 1;
            around = zeros(NMask,MMask);
            for xMask = 1 : NMask
                for yMask = 1 : MMask
                    // Get the pixel of the mask we are using 
                    maskPixel = mask(xMask,yMask);
                    // Get the value of the image we need to convoluate
                    imgPixel = tempMatrix(x2 + xMask, y2 + yMask);
                    // Debug
                    //disp(maskPixel)
                    //disp(imgPixel)
                    // Store result of partial convolution on an array
                    around(xMask,yMask) = maskPixel * imgPixel
                end
            end
            matrixResult(i - halfSizeX,j - halfSizeY) = sum(around);
        end 
    end
endfunction

// Debug
//m=[4,1,2,9,8;3,3,1,3,7;4,7,6,5,2;4,8,3,7,1;3,7,7,9,3]
//mask=[1,2,1;2,4,2;1,2,1]
