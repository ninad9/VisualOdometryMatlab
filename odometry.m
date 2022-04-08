% Reading images of left and right cameras%
imgFolderLeft = 'E:\FHDortmund\Sem2\CV\Case study\img_undistort\left\';
leftImgs = imageDatastore(fullfile(imgFolderLeft, '*.png'));

imgFolderRight = 'E:\FHDortmund\Sem2\CV\Case study\img_undistort\right\';
rightImgs = imageDatastore(fullfile(imgFolderRight, '*.png'));
%read groundtruth csv file
csvdata = readtable('groundtruth.csv');
 
%looping images%
for i=940:1000
    %i=940;
   imgLeft = readimage(leftImgs,i) ;
   imgRight = readimage(rightImgs,i) ;
   imgLeft_N = readimage(leftImgs,i+1) ;
   imgRight_N = readimage(rightImgs,i+1) ;
   
   %Rectify images
   [oL,oR] = rectifyStereoImages(imgLeft,imgRight,stereoParams);

   %get features of the images
    pts1 = detectHarrisFeatures(oL);
    pts2 = detectHarrisFeatures(oR);
    [f1,vp1] = extractFeatures(oL,pts1);
    [f2,vp2] = extractFeatures(oR,pts2);
    indexPairs = matchFeatures(f1,f2);
    mp1 = vp1(indexPairs(:,1),:);
    mp2 = vp2(indexPairs(:,2),:);
    figure (1);
    title('Feature map');
    showMatchedFeatures(oL,oR,mp1,mp2);
    
    
    P1=mp1.Location;
    P2 = mp2.Location;

    %using RANSAC to get fundamental matrix and inliers
    [F,in,r] = ransac(@fmatrix, [P1'; P2'], 1e-1, 'verbose');
    % idisp({oL, oR}, 'dark');
    
     %inliers and points
     P1n = P1(in,1:2)
     P2n = P2(in,1:2)
     
   
    di = P1n(:,1)-P2n(:,1);
   % stdisp(oL,oR)
    disparitymap = disparity(oL, oR, 'BlockSize', 5);
%     figure (2); hold on;
%     imshow(disparitymap, [-12, 12]);
%     title('Disparity map');
%     colorbar
    
    u0 = K1(1,3);
    v0 = K1(2,3);
    uc1 = round(P1(:,1));
    vc1 = round(P1(:,2));
    b = 0.0796159430046924;
    Z = 203.8.* b./ di;
    X = b.*(uc1-u0)./ di;
    Y = b.*(vc1-v0)./ di;
    
    %calculate pose
    Tr = icp(P1n', P2n');
    pose = inv(Tr(1))*T12;
    
    
    
    %pose from ground truth data
    if(i>=940 && i<=1000)
        tx = csvdata.Var3(i);
        ty = csvdata.Var4(i);
        tz = csvdata.Var5(i);
        qx = csvdata.Var6(i);
        qy = csvdata.Var7(i);
        qz = csvdata.Var8(i);
        qw = csvdata.Var9(i);
        pose2 = SE3(tx,ty,tz)*SE3(qx,qy,qz);
     end
    %calculate translation error
    %error = pose - pose2;
end
