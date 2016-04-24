// FUNCTIONS

// Don't show warnings
funcprot(0)

// \fn imgMatrix
// \brief Create matrix from image
// \return imgMatrix: image loaded
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
function writeImage(imgMatrix,fileName)
    image = imwrite(imgMatrix,fileName);
endfunction

// \fn applyMask
// \brief Apply a given filter to an image
// \args imgMatrix: image to treat, mask: mask to use
// \return matrixResult: image which has been treated
function matrixResult = applyMask(imgMatrix,mask)
    // Normalize the mask if the sum of all the components of the mask is different of 0
    if (sum(mask) ~= 0) then
        mask = (1 / sum(mask)).*mask;
    end
    // Get size of the matrix
    [N, M] = size(imgMatrix); 
    // Get size of the mask 
    [NMask, MMask] = size(mask);
    // Get the halfSize of the mask to avoid problems in the edges of the picture
    halfSizeX = floor(NMask / 2);      
    halfSizeY = floor(MMask / 2);

    XsizeOfTempMatrix = N + 2 * halfSizeX;
    YsizeOfTempMatrix = M + 2 * halfSizeY;

    // Initialise temp bigger matrix for treatment
    tempMatrix = zeros(XsizeOfTempMatrix,YsizeOfTempMatrix);
    [Ntemp, Mtemp] = size(tempMatrix);

    // Debug: before treatment
    // disp(imgMatrix);
    // disp(tempMatrix);

    tempMatrix(halfSizeX + 1 : Ntemp - halfSizeX,halfSizeY + 1 : Mtemp - halfSizeY) = imgMatrix;

    // Debug
    // disp(tempMatrix);
    // disp(imgMatrix(1,1));
    // disp(tempMatrix(1,1)); // Should print 0 0
    // disp(tempMatrix(halfSizeX+1,halfSizeY+1)); // Should print same as disp(imgMatrix(1,1)) 

    // Init matrixResult
    matrixResult = zeros(N,M)

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
                    // Get the value of the image we need to convolute
                    imgPixel = tempMatrix(x2 + xMask, y2 + yMask);
                    // Debug
                    // disp(maskPixel);
                    // disp(imgPixel);
                    // Store result of partial convolutions on an array
                    around(xMask,yMask) = maskPixel * imgPixel;
                end
            end
            matrixResult(i - halfSizeX,j - halfSizeY) = sum(around);
        end 
    end
endfunction

// \fn gradientNorm
// \brief Apply the gradients to picture
// \args imgMatrix: image to treat
// \return Es: gradients, Eo: gradient angle
function [Es,Eo] = gradientNorm(imgMatrix)
    // Get size of the matrix
    [N, M] = size(imgMatrix);

    // Init the masks for convolution
    mask1 = [1,0,-1];
    mask2 = [1;0;-1];

    // Apply masks
    Jx = applyMask(imgMatrix,mask1);
    Jy = applyMask(imgMatrix,mask2);

    // Apply the given mathematical formulas to create Es and Eo
    for i = 1 : N
        for j = 1 : M
            Es(i,j) = sqrt(Jx(i,j)^2 + Jy(i,j)^2);
            //Eo(i,j) = normalizeAngle(atan(-Jx(i,j),Jy(i,j))*180/%pi);
            Eo(i,j) = normalizeAngle(atan(-Jx(i,j),Jy(i,j))*180/%pi);
        end
    end
endfunction

// \fn deleteNonMax
// \brief Delete the non-maximum in the Es matrix (using Eo)
// \args Es: matrix to treat, Eo: matrix to use to follow gradient norm
// \return imgWithoutMax
function imgWithoutMax = deleteNonMax(Es,Eo)
    [N,M] = size(Es);

    for i = 1 : N
        for j = 1 : M
            // Save the actual angle of the gradient
            gradientAngle = Eo(i,j);
            // Save the actual value
            actualValue = Es(i,j);
            // Get the coord of the neighbors
            [xTemp1,yTemp1,xTemp2,yTemp2] = getNeighborhoodCoords(gradientAngle);

            // Check if the pixels we want to compare exists in Eo
            firstValueToCompare = getMatValueIfExists(xTemp1,yTemp1,Es,N,M);
            secondValueToCompare = getMatValueIfExists(xTemp2,yTemp2,Es,N,M);

            // And then, delete the non maximums
            if (actualValue < firstValueToCompare) | (actualValue < secondValueToCompare) then
                imgWithoutMax(i,j) = 0;
            else
                imgWithoutMax(i,j) = actualValue;
            end
        end
    end 
endfunction

// \fn computeThreshold
// \brief Compute the threshold automatically (mimic of perctl)
// \args img: img to use to compute threshold, perc: percentage we will use to
//       create threshold, histSize: size of histogramIndexes we will need
// \return Th: high threshold, Tl: low threshold
function [Th,Tl] = computeThreshold(img, perc, histSize)
    // Size of image
    [N,M] = size(img);
    // Normalize percentage
    perc = perc / 100;
  
    // Normalise matrix to delete floating numbers...
    for i = 1 : N
        for j = 1 : M
            normalizedMatrix(i,j) = floor(img(i,j));
        end
    end

    // Calculate the steps
    valueMax = max(normalizedMatrix);
    valueMin = min(normalizedMatrix);
    step = (valueMax - valueMin) / histSize;

    // histSize is the number of value we want to put on our histogram
    histogramIndexes = zeros(1, histSize + 1);
    histogram = zeros(1, histSize + 1);

    for i = 2 : size(histogramIndexes,2)
        histogramIndexes(i) = ((step * i) - step);
    end

    for i = 1 : N
        for j = 1 : M
            actVal = normalizedMatrix(i,j);
            index = floor((actVal - valueMin) / step) + 1;
            
            histogram(index) = histogram(index) + 1;
        end
    end

    // Normalisation of histogram
    normalizedHistogram = histogram / sum(histogram);

    // Debug
    // disp(histogramIndexes);
    // disp(normalizedHistogram);

    // Histogram
    subplot(121);
    plot(histogramIndexes,normalizedHistogram);

    // Repartition
    repartition = cumsum(normalizedHistogram);
    subplot(122);
    plot(repartition);

    indexOfPercentile = 1;
    while repartition(indexOfPercentile) < perc
        indexOfPercentile = indexOfPercentile + 1;
    end

    // Debug
    // disp(histogramIndexes(indexOfPercentile));

    Th = histogramIndexes(indexOfPercentile);
    Tl = Th / 2;
endfunction

// \fn hysteresisThreshold
// \brief Apply the hysteresis threshold on the image 
// \args img: img to treat, Eo: gradient angle matrix associated to img,
//       perc: percentage we will use to compute the threshold Th
function thresholdedImage = hysteresisThreshold(img,Eo,Es,perc)
    // Debug
    // 70% of the pixels in the image are below p value
    // perctl is used to compute Th automatically
    // p = perctl(Es,70);
    // Th = (p(1));

    [Th,Tl] = computeThreshold(Es,perc,100);
    [N,M] = size(img);

    // Init matrices for treatment
    tempThresholdedImage = zeros(N,M);
    thresholdedImage = zeros(N,M);

    // First iteration
    for i = 1 : N
        for j = 1 : M
            // Three cases:  img(i,j) > Th, img(i,j) < Tl, Th > img(i,j) > Tl
            if img(i,j) > Th then
                tempThresholdedImage(i,j) = 255;
            elseif img(i,j) < Tl then
                tempThresholdedImage(i,j) = 0;
            end          
        end
    end

    // Second iteration
    for i = 1 : N
        for j = 1 : M
            gradPerp = normalizeAngle(Eo(i,j) + 90);
            [xTemp1,yTemp1,xTemp2,yTemp2] = getNeighborhoodCoords(gradPerp);

            // Check if the pixels we want to compare exists in Eo
            firstValueToCompare = getMatValueIfExists(xTemp1,yTemp1,tempThresholdedImage,N,M);
            secondValueToCompare = getMatValueIfExists(xTemp2,yTemp2,tempThresholdedImage,N,M);

            // Check the neighbor pixel to see if there is an edge
            if (firstValueToCompare == 255) | (secondValueToCompare == 255) then
                thresholdedImage(i,j) = 255;
            else
                thresholdedImage(i,j) = 0;
            end       
        end
    end
endfunction

// \fn normalizeAngle
// \brief Normalize angle of the norm of the gradient
// \args angle
// \return normalizedAngle
function normalizedAngle = normalizeAngle(angle)
    if (angle >= 157.5) then
        normalizedAngle = normalizeAngle(angle - 180);
    elseif (angle < -22.5) then
        normalizedAngle = normalizeAngle(angle + 180);    
    elseif (angle >= 22.5) & (angle < 67.5) then
        normalizedAngle = 45;
    elseif (angle >= 67.5) & (angle < 112.5) then
        normalizedAngle = 90;
    elseif (angle >= 112.5) & (angle < 157.5) then
        normalizedAngle = 135;
    elseif (angle >= -22.5) & (angle < 22.5) then
        normalizedAngle = 0;
    end
endfunction 

// \fn concateneImg
// \brief concatene four images to display them in only one window
// \args four img to concat
// \return y: cat of all the 4 img provided
function y = concateneImg(img1,img2,img3,img4)
    y = cat(2,img1,img2,img3,img4);
endfunction

// \fn getNeighborhoodCoords
// \brief Get the coords of the two neighbours of the pixel, following the gradient angle
// \args gradientAngle: angle in deg
// \return xTemp1,yTemp1,xTemp2,yTemp2: coordinates
function [xTemp1,yTemp1,xTemp2,yTemp2] = getNeighborhoodCoords(gradientAngle)
    select gradientAngle
        case 0 then
            xTemp1 = i - 1;
            yTemp1 = j;
            xTemp2 = i + 1;
            yTemp2 = j;
        case 45 then
            xTemp1 = i - 1;
            yTemp1 = j + 1;
            xTemp2 = i + 1;
            yTemp2 = j - 1;
        case 90 then
            xTemp1 = i;
            yTemp1 = j - 1;
            xTemp2 = i;
            yTemp2 = j + 1;
        case 135 then
            xTemp1 = i - 1;
            yTemp1 = j - 1;
            xTemp2 = i + 1;
            yTemp2 = j + 1;
        else
            break;
    end
endfunction

// \fn getMatValueIfExists
// \brief Get a value by index, if index is in range
// \args x: coordX to test, y: coordY to test, mat: matrice where we want to extract value,
//       N: size X of matrix, M: size Y of matrix
// \return value: value in the matrix, if it exists
function value = getMatValueIfExists(x,y,mat,N,M)
    if (x > 0) & (x < N + 1) & (y > 0) & (y < M + 1) then
        value = mat(x,y);
    else
        value = 0;
    end
endfunction

// TESTS

// \fn testApplyMask
// \brief Test the function "applyMask"
// \args imgMatrix: image to treat, mask: mask to use
// \return resTest: result of the test
function resTest = testApplyMask(imgMatrix,mask)
    // We should give to imfilter a normalized mask !
    normalizedMask = (1 / sum(mask)).*mask;

    // Init result matrix to compare output of my function to the official one
    matrixResult = applyMask(imgMatrix,mask);
    // Init test matrix
    matrixTest = imfilter(imgMatrix,normalizedMask);
    
    // With border efects because imfilter works differently concerning the edges
    resTest = matrixResult - matrixTest;
    // Debug: should be full of 0
    // disp(resTest)
endfunction

// MAIN

function main()
    // For big pictures, increase size of stack
    stacksize('max');

    // Load image
    img = loadImage('X:\ENSSAT\IMR2\S4\TRAITEMENT_IMAGE\PROJET\img1.jpg',1);
    // Init mask
    // mask = [1,2,1;2,4,2;1,2,1];
    mask = [2,4,5,4,2;4,9,12,9,4;5,12,15,12,5;4,9,12,9,4;2,4,5,4,2];
    // Apply the mask, step 1: gaussian filter
    filteredImg = applyMask(img,mask);
    // Test
    testApplyMask(img,mask);

    // Compute the gradient norm 
    [Es,Eo] = gradientNorm(filteredImg);
    // Remove max from img
    imgWithoutMax = deleteNonMax(Es,Eo);
    // Apply hysteresisThreshold
    thresholdedImage = hysteresisThreshold(imgWithoutMax,Eo,Es,86);

    // Display result of main
    resultImage = concateneImg(img,filteredImg,imgWithoutMax,thresholdedImage);
    displayImage(resultImage);

    // Write result
    //writeImage(uint8(resultImage),'X:\ENSSAT\IMR2\S4\TRAITEMENT_IMAGE\PROJET\img3_res_86.jpg');
endfunction

main()
