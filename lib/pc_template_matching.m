function [center_fitted, D_fitted, fitted_template_pts] = pc_template_matching(apple_pc,template_pc,D_template,maxDistance)

    D_to_fit = (0.02:0.002:0.096);    
    apple_center = mean(apple_pc.Location);
    apple_pc_DS = pcdownsample(apple_pc,'gridAverage',0.005);
    apple_pc_t = pointCloud(apple_pc_DS.Location - apple_center);
    max_inliers=0;
    Options.Verbose=0;
    for D = D_to_fit
        %disp(strcat('Fitting a template of diameter ', ' ', num2str(D)));
        template_pc_s = template_pc.Location.*(D/D_template); 
        template_center =  mean(template_pc_s(:,1:3));
        template_pc_s_t = template_pc_s(:,1:3) - template_center;
        template_ptCloud_DS = pcdownsample(pointCloud(template_pc_s_t),'gridAverage',0.005);
        
        [Points_Moved,M]=ICP_finite( template_ptCloud_DS.Location,apple_pc_t.Location,Options);
        [~,Dist] = knnsearch(template_ptCloud_DS.Location,Points_Moved);
        num_inliers=sum(Dist<maxDistance);
        if num_inliers>max_inliers
            max_inliers=num_inliers;
            center_fitted = apple_center - M(1:3,4)';
            fitted_template_pts=template_ptCloud_DS.Location + center_fitted;
            D_fitted = D;

        end
    end
end
    
    
    
    
    
    
    
    

