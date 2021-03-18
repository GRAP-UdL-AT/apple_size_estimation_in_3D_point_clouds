function [occlusion,GT_sphere_pts, Visible_GTpts] = compute_fruit_occlusion(ptCloud,r,sphereCenter,icospherIdx)

    [V,~] = icosphere(icospherIdx);
    GT_sphere_pts = V.*r + sphereCenter;
    distGT = sqrt(sum((GT_sphere_pts(1,:)-GT_sphere_pts(2,:)).^2));
    ptCloud_down = pcdownsample(ptCloud,'gridAverage',distGT/3);
    Visible_GTpts = unique(knnsearch(GT_sphere_pts,ptCloud_down.Location));
    occlusion = size(Visible_GTpts,1) / size(GT_sphere_pts,1);
    
end
