function [thresh, metric] = mymultithresh(varargin)
%此为MATLAB自带函数，略作修改
%MULTITHRESH Multi-level image thresholds using Otsu's method.
%   THRESH = MULTITHRESH(A) computes a single threshold for image A using
%   Otsu's method and returns it in THRESH. THRESH can be used to convert A
%   into a two-level image using IMQUANTIZE.
%
%   THRESH = MULTITHRESH(A, N) computes N thresholds for image A using the
%   Otsu's method and returns them in THRESH. THRESH is a 1xN vector which
%   can be used to convert A into an image with (N+1) discrete levels using
%   IMQUANTIZE. The maximum value allowed for N is 20.
%
%   [THRESH, METRIC] = MULTITHRESH(A,...) returns the effectiveness metric
%   as the second output argument. METRIC is in the range [0 1] and a
%   higher value indicates greater effectiveness of the thresholds in
%   separating the input image into N+1 regions based on Otsu's objective
%   criterion. 
%
%   Class Support
%   -------------
%   The input image A is an array of one of the following classes: uint8,
%   uint16, int16, single, or double. It must be nonsparse. N is a positive
%   integer between 1 and 20, and can be any numeric class. THRESH is a 1xN
%   numeric vector of the same class as A. METRIC is a scalar of class
%   double.
%
%   Notes
%   -----
%   1. A can be an array of any dimension. MULTITHRESH finds the thresholds
%      based on the aggregate histogram of the entire array. An RGB image
%      is considered a 3D numeric array and the thresholds are computed
%      for the combined data from all the three color planes.
%
%   2. MULTITHRESH uses the range of the input A [min(A(:)) max(A(:))] as
%      the limits for computing image histogram which is used in subsequent
%      computations. Any NaNs in A are ignored in computation. Any Infs and
%      -Infs in A are counted in the first and last bin of the histogram,
%      respectively.
%
%   3. For N > 2, MULTITHRESH uses search-based optimization of Otsu's
%      criterion to find the thresholds. The search-based optimization
%      guarantees only locally optimal results. Since the chance of
%      converging to local optimum increases with N, it is preferrable to
%      use smaller values of N, typically N < 10.
%
%   4. For degenerate inputs where the number of unique values in A is less
%      than or equal to N, there is no viable solution using Otsu's method.
%      For such inputs, MULTITHRESH returns THRESH such that it includes
%      all the unique values from A and possibly some extra values that are
%      chosen arbitrarily.
% 
%   5. The thresholds (THRESH) returned by MULTITHRESH are in the same
%      range as the input image A. This is unlike GRAYTHRESH which returns
%      a normalized threshold in the range [0, 1].
%
%   Example 1
%   ---------
%   This example computes multiple thresholds for an image using
%   MULTITHRESH and applies those thresholds to the image using IMQUANTIZE
%   to get segment labels.
%
%     I = imread('circlesBrightDark.png');
%     imshow(I, [])
%     title('Original Image');
%
%     % Compute the thresholds
%     thresh = multithresh(I,2);
%
%     % Apply the thresholds to obtain segmented image
%     seg_I = imquantize(I,thresh);
%
%     % Show the various segments in the segmented image in color
%     RGB = label2rgb(seg_I);
%     figure, imshow(RGB)
%     title('Segmented Image');
%
%   See also GRAYTHRESH, IM2BW, IMQUANTIZE, RGB2IND.

%   Copyright 2012-2014 The MathWorks, Inc.

% Reference
% ---------
% N. Otsu, "A Threshold Selection Method from Gray-Level Histograms," IEEE
% Transactions on Systems, Man, and Cybernetics, Vol. 9, No. 1, pp. 62-66,
% 1979.

narginchk(1,2);

[A, N] = parse_inputs(varargin{:});

if (isempty(A))
    warning(message('images:multithresh:degenerateInput',N))
    thresh = getDegenerateThresholds(A, N);    
    metric = 0.0;
    return;
end

% Variables are named similar to the formulae in Otsu's paper.
num_bins = 256;

[p, minA, maxA] = getpdf(A, num_bins);

if (isempty(p))
    % Input image pdf could not be computed
    warning(message('images:multithresh:degenerateInput',N))
    thresh = getThreshForNoPdf(minA, maxA, N);        
    metric = 0.0;
    return;
end

omega = cumsum(p);
mu = cumsum(p .* (1:num_bins)');
mu_t = mu(end);

if (N < 3)   
    
    sigma_b_squared = calcFullObjCriteriaMatrix(N, num_bins, omega, mu, mu_t); 
    
    % Find the location of the maximum value of sigma_b_squared.  
    maxval = max(sigma_b_squared(:));     
    isvalid_maxval = isfinite(maxval);
    
    if isvalid_maxval
        % Find the bin with maximum value. If the maximum extends over
        % several bins, average together the locations.
        switch N
            case 1
                idx = find(sigma_b_squared == maxval);
                % Find the intensity associated with the bin
                thresh = mean(idx) - 1;
            case 2
                [maxR, maxC] = find(sigma_b_squared == maxval);
                % Find the intensity associated with the bin
                thresh = mean([maxR maxC],1) - 1;                
        end        
    else
        [isDegenerate, uniqueVals] = checkForDegenerateInput(A, N);
        if isDegenerate
            warning(message('images:multithresh:degenerateInput',N));
        else
            warning(message('images:multithresh:noConvergence'));
        end
        thresh = getDegenerateThresholds(uniqueVals, N);
        metric = 0.0;        
    end
    
else
    
    % For N >= 3, use search-based optimization of Otsu's objective function
    
    % Set initial thresholds as uniformly spaced
    initial_thresh = linspace(0, num_bins-1, N+2);
    initial_thresh = initial_thresh(2:end-1); % Retain N thresholds
   
    % Set optimization parameters    
    options = optimset('TolX',1,'Display','off');    
    % Find optimum using fminsearch 
    [thresh, minval] = fminsearch(@(thresh) objCriteriaND(thresh, ...
        num_bins, omega, mu, mu_t), initial_thresh, options);
    
    maxval = -minval;
    
    isvalid_maxval = ~(isinf(maxval) || isnan(maxval));
    if isvalid_maxval        
        thresh = round(thresh);  
    end
            
end

% Prepare output values
if isvalid_maxval
    
    % Map back to original scale as input A
    thresh = map2OriginalScale(thresh, minA, maxA);
    if nargout > 1    
        % Compute the effectiveness metric        
        metric = maxval/(sum(p.*(((1:num_bins)' - mu_t).^2)));        
    end
    
else
    
    [isDegenerate, uniqueVals] = checkForDegenerateInput(A, N);  
    if isDegenerate
        warning(message('images:multithresh:degenerateInput',N));
        thresh = getDegenerateThresholds(uniqueVals, N);
        metric = 0.0;
    else
        warning(message('images:multithresh:noConvergence'));
        % Return latest available solution
        thresh = map2OriginalScale(thresh, minA, maxA);
        if nargout > 1
            % Compute the effectiveness metric
            metric = maxval/(sum(p.*(((1:num_bins)' - mu_t).^2)));
        end
    end
        
end

end

%--------------------------------------------------------------------------

function [A, N] = parse_inputs(varargin)

A = varargin{1};
validateattributes(A,{'uint8','uint16','int16','double','single'}, ...
    {'nonsparse', 'real'}, mfilename,'A',1);

if (nargin == 2)
    N = varargin{2};
    validateattributes(N,{'numeric'},{'integer','scalar','positive','<=',20}, ...
        mfilename,'N',2);
else
    N = 1; % Default N
end
end

%--------------------------------------------------------------------------

function [p, minA, maxA] = getpdf(A,num_bins)

% Vectorize A for faster histogram computation
A = A(:);

if isfloat(A)    
    % If A is an float images then scale the data to the range [0 1] while
    % taking care of special cases such as Infs and NaNs.
    
    % Remove NaNs from consideration.
    
    % A cannot be empty here because we checked for it earlier.  
    A(isnan(A)) = [];   
    if isempty(A)
        % The case when A was full of only NaNs.
        minA = NaN;
        maxA = NaN;
        p = [];
        return;
    end    
    
    % Scale A to [0-1]
    idxFinite = isfinite(A);
    % If there are finite elements, then scale them between [0-1]. Maintain
    % Infs and -Infs as is so that they get included in the pdf.
    if any(idxFinite)
        minA = min(A(idxFinite));
        maxA = max(A(idxFinite));
        if(minA == maxA)
            p = [];
            return;
        end        
        % Call to BSXFUN below is equivalent to A = (A - minA)/(maxA - minA);
        A = bsxfun(@rdivide,bsxfun(@minus, A, minA),maxA - minA);
        
    else
        % One of many possibilities: all Infs, all -Infs, mixture of Infs
        % and -Infs, mixture of Infs with NaNs.
        minA = min(A);
        maxA = max(A);
        p = [];
        return;
    end
else
    % If A is an integer image then no need to handle special cases for
    % Infs and NaNs.    
    minA = min(A);
    maxA = max(A);
    if(minA == maxA)
        p = [];
        return;
    else
        % Call to BSXFUN below is equivalent to A = single(A - minA)./single(maxA - minA);      
        A = bsxfun(@rdivide,single(bsxfun(@minus, A, minA)),single(maxA - minA));
    end
    
end

% Convert to uint8 for fastest histogram computation.
A = grayto8mex(A);
counts = imhist(A,num_bins);
counts(1) = 0;%忽略0强度值
p = counts / sum(counts);

end

%--------------------------------------------------------------------------

function sigma_b_squared_val = objCriteriaND(thresh, num_bins, omega, mu, mu_t)

% 'thresh' has intensities [0-255], but 'boundaries' are the indices [1
% 256].
boundaries = round(thresh)+1; 

% Constrain 'boundaries' to:
% 1. be strictly increasing, 
% 2. have the lowest value > 1 (i.e. minimum 2), 
% 3. have highest value < num_bins (i.e. maximum num_bins-1).
if (~all(diff([1 boundaries num_bins]) > 0))
    sigma_b_squared_val = Inf;
    return;
end

boundaries = [boundaries num_bins]; 

sigma_b_squared_val = omega(boundaries(1)).*((mu(boundaries(1))./omega(boundaries(1)) - mu_t).^2);

for kk = 2:length(boundaries)
    omegaKK = omega(boundaries(kk)) - omega(boundaries(kk-1));
    muKK = (mu(boundaries(kk)) - mu(boundaries(kk-1)))/omegaKK;
    sigma_b_squared_val = sigma_b_squared_val + (omegaKK.*((muKK - mu_t).^2)); % Eqn. 14 in Otsu's paper
end

if (isfinite(sigma_b_squared_val))
    sigma_b_squared_val = -sigma_b_squared_val; % To do maximization using fminsearch.
else
    sigma_b_squared_val = Inf;
end
end

%--------------------------------------------------------------------------

function sigma_b_squared = calcFullObjCriteriaMatrix(N, num_bins, omega, mu, mu_t)
if (N == 1)
    
    sigma_b_squared = (mu_t * omega - mu).^2 ./ (omega .* (1 - omega));
    
elseif (N == 2)
    
    % Rows represent thresh(1) (lower threshold) and columns represent
    % thresh(2) (higher threshold).
    omega0 = repmat(omega,1,num_bins);
    mu_0_t = repmat(bsxfun(@minus,mu_t,mu./omega),1,num_bins);
    omega1 = bsxfun(@minus, omega.', omega);
    mu_1_t = bsxfun(@minus,mu_t,(bsxfun(@minus, mu.', mu))./omega1);
    
    % Set entries corresponding to non-viable solutions to NaN
    [allPixR, allPixC] = ndgrid(1:num_bins,1:num_bins); 
    pixNaN = allPixR >= allPixC; % Enforce thresh(1) < thresh(2)
    omega0(pixNaN) = NaN;
    omega1(pixNaN) = NaN;
          
    term1 = omega0.*(mu_0_t.^2);
    
    term2 = omega1.*(mu_1_t.^2);
    
    omega2 = 1 - (omega0+omega1);
    omega2(omega2 <= 0) = NaN; % Avoid divide-by-zero Infs in term3
    
    term3 = ((omega0.*mu_0_t + omega1.*mu_1_t ).^2)./omega2;
    
    sigma_b_squared = term1 + term2 + term3;
end
end

%--------------------------------------------------------------------------

function sclThresh = map2OriginalScale(thresh, minA, maxA)

normFactor = 255;
sclThresh = double(minA) + thresh/normFactor*(double(maxA) - double(minA));
sclThresh = cast(sclThresh,'like',minA);

end

%--------------------------------------------------------------------------
function [isDegenerate, uniqueVals] = checkForDegenerateInput(A, N)

uniqueVals = unique(A(:))'; % Note: 'uniqueVals' is returned in sorted order. 

% Ignore NaNs because they are ignored in computation. Ignore Infs because
% Infs are mapped to extreme bins during histogram computation and are
% therefore not unique values.
uniqueVals(isinf(uniqueVals) | isnan(uniqueVals)) = []; 

isDegenerate = (numel(uniqueVals) <= N);

end

%--------------------------------------------------------------------------
function thresh = getThreshForNoPdf(minA, maxA, N)

if isnan(minA)
    % If minA = NaN => maxA = NaN. All NaN input condition.
    minA = 1; 
    maxA = 1;
end

if (N == 1)
    thresh = minA;
else
    if (minA == maxA)
        % Flat image, i.e. only one unique value (not counting Infs and
        % -Infs) exists
        thresh = getDegenerateThresholds(minA, N);
    else
        % Only scenario: A full of Infs and -Infs => minA = -Inf and maxA =
        % Inf
        thresh = getDegenerateThresholds([minA maxA], N);
    end
end

end

%--------------------------------------------------------------------------
function thresh = getDegenerateThresholds(uniqueVals, N)
% Notes:
% 1) 'uniqueVals' must be in sorted (ascending) order
% 2) For predictable behavior, 'uniqueVals' should not have NaNs 
% 3) For predictable behavior for all datatypes including uint8, N must be < 255

if isempty(uniqueVals)
    thresh = cast(1:N,'like', uniqueVals);
    return;
end

% 'thresh' will always have all the elements of 'uniqueVals' in it.
thresh = uniqueVals;

thNeeded1 = N - numel(thresh);
if (thNeeded1 > 0)
    
    % More values are needed to fill 'thresh'. Start filling 'thresh' from
    % the lower end starting with 1.
    
    if (uniqueVals(1) > 1)
        % If uniqueVals(1) > 1, we can directly fill some (or maybe all)
        % values starting from 1, without checking for uniqueness.        
        thresh = [cast(1:min(thNeeded1,ceil(uniqueVals(1))-1), 'like', uniqueVals)...
            thresh];        
    end
    
    thNeeded2 = N - numel(thresh);
    if (thNeeded2  > 0) 
        
        % More values are needed to fill 'thresh'. Use positive integer
        % values, as small as possible, which are not in 'thresh' already.
        lenThreshOrig = length(thresh);               
        thresh = [thresh zeros(1,thNeeded2)]; % Create empty entries, thresh datatype presevrved
        uniqueVals_d = double(uniqueVals); % Needed to convert to double for correct uniquness check     
        threshCandidate = max(floor(uniqueVals(1)),0); % Always non-negative, threshCandidate datatype presevrved    
        q = 1;
        while q <= thNeeded2
            threshCandidate = threshCandidate + 1;
            threshCandidate_d = double(threshCandidate); % Needed to convert to double for correct uniquness check
            if any(abs(uniqueVals_d - threshCandidate_d) ...
                    < eps(threshCandidate_d)) 
                % The candidate value already exists, so don't use it.               
                continue;
            else
                thresh(lenThreshOrig + q) = threshCandidate; % Append at the end
                q = q + 1;
            end
        end
        
        thresh = sort(thresh); % Arrange 'thresh' in ascending order
        
    end
                             
end
    
end
