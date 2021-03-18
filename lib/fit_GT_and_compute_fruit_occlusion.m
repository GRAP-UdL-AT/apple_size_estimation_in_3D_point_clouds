function [occlusion,GT_sphere_pts, Visible_GTpts] = fit_GT_and_compute_fruit_occlusion(ptCloud,r,icospherIdx)

    points = ptCloud.Location;
    sampleSize = 3; % number of points to sample per trial
    maxDistance = 0.005; % max allowable distance for inliers
    
    sphere_fitted=0;
    while sphere_fitted==0
        try
            [sphereCenterRANSAC, ~] = fit_sphere_given_radius_RANDSAC(points,r,sampleSize,maxDistance);
            sphere_fitted=1;
        catch
            maxDistance = maxDistance + 0.005;
            warning(strcat('Not enought inliers to fit an sphere. Max Distance increased to ', num2str(maxDistance),'m'));
        end
    end

    [V,~] = icosphere(icospherIdx);
    GT_sphere_pts = V.*r + sphereCenterRANSAC;
    distGT = sqrt(sum((GT_sphere_pts(1,:)-GT_sphere_pts(2,:)).^2));
    ptCloud_down = pcdownsample(ptCloud,'gridAverage',distGT/3);
    Visible_GTpts = unique(knnsearch(GT_sphere_pts,ptCloud_down.Location));
    occlusion = size(Visible_GTpts,1) / size(GT_sphere_pts,1);
    
end
