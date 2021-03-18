function [sphereCenterRANSAC, inlierIdx] = fit_sphere_given_radius_RANDSAC(points,r,sampleSize,maxDistance)

    f = @(C) sum((sum(bsxfun(@minus,points,C).^2,2)-r^2).^2);
    fitSphereFcn = @(points) fminsearch(f, mean(points));
    evalSphereFcn = @(model, points) abs(sqrt(sum(bsxfun(@minus,points,model).^2,2))-r);
    [sphereCenterRANSAC, inlierIdx] = ransac(points,fitSphereFcn,evalSphereFcn, sampleSize,maxDistance,'MaxNumTrials',3000);
    
    
end
