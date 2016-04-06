// FUNCTIONS

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
    // Normalize the mask if the sum of all the components of the mask
    // is different of 0
    if (sum(mask) ~= 0) then
        mask = (1/sum(mask)).*mask;
    end
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

function [Es,Eo] = gradiantNorm(imgMatrix)
    // Get size of the matrix
    [N, M] = size(imgMatrix);

    // Init the masks for convolution
    mask1 = [1,0,-1];
    mask2 = [1;0;-1];

    // Apply masks...
    Jx = applyMask(imgMatrix,mask1);
    Jy = applyMask(imgMatrix,mask2);

    for i = 1 : N
        for j = 1 : M
            Es(i,j) = sqrt(Jx(i,j)^2 + Jy(i,j)^2);
            Eo(i,j) = normalizeAngle(atan(-Jx(i,j) / Jy(i,j)));
        end
    end
    disp(Es);
    disp(Eo);
endfunction

// \fn normalizeAngle
// \brief Normalize angle of the norm of the gradient
// \args angle
// \return normalizedAngle
function normalizedAngle = normalizeAngle(angle)
    if (angle >= -22.5) & (angle <= 22.5) then
        normalizedAngle = 0;
    elseif (angle >= 22.5) & (angle <= 67.5) then
        normalizedAngle = 45;
    elseif (angle >= 67.5) & (angle <= 112.5) then
        normalizedAngle = 90;
    elseif (angle <= 112.5) & (angle >= 157.5) then
        normalizedAngle = 135;
    end,
endfunction

// TESTS

// \fn testApplyMask
// \brief Test the function "applyMask"
// \args imgMatrix: image to treat, mask: mask to use
// \return resTest: result of the test
function resTest = testApplyMask(imgMatrix,mask)
    // Init result matrix to compare output of my function to the official one
    matrixResult = applyMask(imgMatrix,mask);
    // Init test matrix
    matrixTest = imfilter(imgMatrix,mask);
    // Should be full of 0...
    // But it is not -> FIXME
    resTest = matrixResult - matrixTest;
endfunction

// MAIN
function lenna2 = main
    // Load image
    lenna = loadImage('X:\ENSSAT\IMR2\S4\TRAITEMENT_IMAGE\PROJET\lenna.jpeg',1);
    //displayImage(lenna);
    // Init mask
    mask = [1,2,1;2,4,2;1,2,1];
    // Apply the mask, step 1: gaussian filter
    lenna2 = applyMask(lenna,mask);
    displayImage(lenna2);
    //displayImage(lenna);
    gradiantNorm(lenna2);
endfunction

// DEBUG
//m=[4,1,2,9,8;3,3,1,3,7;4,7,6,5,2;4,8,3,7,1;3,7,7,9,3]
//mask=[1,2,1;2,4,2;1,2,1]
