clc
clear

%setting path's
addpath("lib");
code_dir = pwd;
idcs = strfind(code_dir,filesep);
pc_folder = fullfile(code_dir(1:idcs(end)-1),'PFuji-Size_dataset','3-apple_point_clouds');
GT_file = fullfile(code_dir(1:idcs(end)-1),'PFuji-Size_dataset','4-apples_GT_diameter.txt');
template_file=fullfile(code_dir(1:idcs(end)-1),'PFuji-Size_dataset','3-apple_point_clouds','2020_02_001.txt');
output_folder = fullfile(code_dir(1:idcs(end)-1),'results_test');
output_fitted_shapes = fullfile(code_dir(1:idcs(end)-1),'results_test','fitted_shapes');
if ~isfolder(output_folder)
    mkdir(output_folder);
end
if ~isfolder(output_fitted_shapes)
    mkdir(output_fitted_shapes);
end

%configuration
save_results = 0;
save_fitted_pc = 0;
density_radius = 0.005;
maxDistance = 0.005; % max distance to consider a point as inliers
icospherIdx = 4;
[V,F] = icosphere(icospherIdx);
[V3,F3] = icosphere(3);

%diameter ground truth reading
data_list = readcell(GT_file,'Delimiter',{','});

%load template for template matching
D_template = 0.056; %m
template_pts = readmatrix(template_file);
template_pc = pointCloud(template_pts(:,1:3));

%Fruit size and visibility estimation
i=randi(size(data_list,1),1,1);
% for i=1:size(data_list,1)   %uncomment this line and the last 3 lines of this script to process all the dataset

    disp(strcat('Processing apple ', num2str(i), ': ', data_list{i,1}))
    plot_description = data_list{i,1}(1:end-4);
    %% Data reading and pre-processing
    apple_pc = readmatrix(fullfile(pc_folder,data_list{i,1}));
    apple_min_corner = min(apple_pc(:,1:3));
    apple_pc(:,1:3) = apple_pc(:,1:3) - apple_min_corner + [0 0.005 -0.005];
    ptCloud = pointCloud(apple_pc(:,1:3),'Color',apple_pc(:,4:6)./255);
    if size(apple_pc,2) > 8
        ptCloud_filtered = pointCloud(apple_pc(apple_pc(:,8)>1,1:3),'Color',apple_pc(apple_pc(:,8)>1,4:6)./255);
        apple_pc_filtered = apple_pc(apple_pc(:,8)>1,:);
    else
        ptCloud_filtered = ptCloud;
        apple_pc_filtered = apple_pc;
    end
    [ptCloud_filtered2,inlierIndices,~] = pcdenoise(ptCloud_filtered,'NumNeighbors',50);
    apple_pc_filtered2 = apple_pc_filtered(inlierIndices,:);
    r = data_list{i,2}/2000;

    %% Computing fruit occlusion ratio ground truth
    [occlusion,GT_sphere_pts, Visible_GTpts]= fit_GT_and_compute_fruit_occlusion(ptCloud_filtered2,r,icospherIdx);

    %% Mesuring fruits
    %Largest segment
    [D, pairPointsIdx] = pdist2(ptCloud_filtered2.Location,ptCloud_filtered2.Location,'euclidean','largest',1);
    [D_l_seg, maxIdx] = max(D);
    center_l_seg = mean([ptCloud_filtered2.Location(maxIdx,:);ptCloud_filtered2.Location(pairPointsIdx(maxIdx),:)]);
    pts_l_seg = V3.*(D_l_seg/2) + center_l_seg;
    % fit sphere Least squares
    [center_LS,LS_radius] = fit_sphere_LS(ptCloud_filtered2.Location);
    D_LS = LS_radius*2;
    pts_LS = V3.*(D_LS/2) + center_LS;
    % fit sphere MSAC
    [model,inlierIndices,outlierIndices] = pcfitsphere(ptCloud_filtered2,maxDistance,'MaxNumTrials',10000,'Confidence',99.9);
    center_MSAC = model.Center;
    D_MSAC = model.Radius*2;
    pts_MSAC = V3.*(D_MSAC/2) + center_MSAC;
    %fit Template Matching (TM)
    [center_TM, D_TM, pts_TM] = pc_template_matching(ptCloud_filtered2,template_pc,D_template,maxDistance);


    %% Computing fruit occlusion MSAC
    [occlusion_MSAC,MSAC_sphere_pts, Visible_MSACpts]= compute_fruit_occlusion(ptCloud_filtered2,D_MSAC/2,center_MSAC,icospherIdx);

    %% Plot pre-processing
    pointCloud_center = mean(ptCloud_filtered2.Location);
    params=[-30,-10,4,0.1,0.1,0.1];
    point_Cloud_shift = [0.05, 0.05, 0.05];

    ptCloud2= pointCloud(ptCloud.Location + point_Cloud_shift - pointCloud_center,'Color',ptCloud.Color);
    figure;
    pcshow(ptCloud2,'MarkerSize',50);
    setplotlayout("preprocessing",params)
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-01-original_apple_pc.png')), '-dpng', '-r400');
    end

    GT_sphere_pts2 = GT_sphere_pts + point_Cloud_shift - pointCloud_center;
    figure;
    pcshow(ptCloud2,'MarkerSize',50);
    hold on
    patch('Faces',F,'Vertices',GT_sphere_pts2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','None','EdgeColor','black','EdgeAlpha',0.2);
    setplotlayout("preprocessing",params)
    title(strcat('D_GT =',{' '},num2str(data_list{i,2}),'mm','    V =',{' '},num2str(occlusion*100),'%'),'Interpreter','none','Color', 'k')
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-02-GTsphere_fitting.png')), '-dpng', '-r400');
    end
    % 
     ptCloud_filtered3= pointCloud(ptCloud_filtered2.Location + point_Cloud_shift - pointCloud_center,'Color',ptCloud_filtered2.Color);

    figure;
    pcshow(ptCloud_filtered3,'MarkerSize',50);
    %pcshow(ptCloud_filtered2.Location,double(ptCloud_filtered2.Color).*[1 1 0.2]./255,'MarkerSize',10);
    hold on
    VertexColour=repmat([1 0 0],size(GT_sphere_pts2,1),1);
    VertexColour(Visible_GTpts,:)=repmat([0 1 0],size(Visible_GTpts,1),1);
    patch('Faces',F,'Vertices',GT_sphere_pts2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','flat','EdgeColor','black','EdgeAlpha',0.5,'FaceAlpha',0.3,'FaceVertexCData',VertexColour);
    %params(3)=params(3)+0.2;
    title(strcat('D_GT =',{' '},num2str(data_list{i,2}),'mm','    V_GT =',{' '},num2str(occlusion*100),'%'),'Interpreter','none','Color', 'k')
    setplotlayout("preprocessing",params)
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-02-Points_projection_to_GTsphere.png')), '-dpng', '-r400');
    end

    %% Plot MSAC occlusion/visibility
    MSAC_sphere_pts2 = MSAC_sphere_pts + point_Cloud_shift - pointCloud_center;
    figure;
    pcshow(ptCloud_filtered3,'MarkerSize',50);
    %pcshow(ptCloud_filtered2.Location,double(ptCloud_filtered2.Color).*[1 1 0.2]./255,'MarkerSize',10);
    hold on
    VertexColour=repmat([1 0 0],size(MSAC_sphere_pts2,1),1);
    VertexColour(Visible_MSACpts,:)=repmat([0 1 0],size(Visible_MSACpts,1),1);
    patch('Faces',F,'Vertices',MSAC_sphere_pts2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','flat','EdgeColor','black','EdgeAlpha',0.5,'FaceAlpha',0.3,'FaceVertexCData',VertexColour);
    %params(3)=params(3)+0.2;
    title(strcat('D_MSAC =',{' '},num2str(D_MSAC*1000),'mm','    V_MSAC =',{' '},num2str(occlusion_MSAC*100),'%'),'Interpreter','none','Color', 'k')
    setplotlayout("preprocessing",params)
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-02_v2-Points_projection_to_MSACsphere.png')), '-dpng', '-r400');
    end



    %% Plot estimated diameter
    %plot l_seg
    pts_l_seg2 = pts_l_seg + point_Cloud_shift - pointCloud_center;
    sphere_colour = [0.01 0.36 0.59];
    sphere_marker = 'diamond';
    figure;
    pcshow(ptCloud2,'MarkerSize',50);
    hold on
    VertexColour=repmat(sphere_colour,size(pts_l_seg2,1),1);
    patch('Faces',F3,'Vertices',pts_l_seg2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','flat','EdgeColor','black','EdgeAlpha',0.7,'FaceAlpha',0.6,'FaceVertexCData',VertexColour);
    %pcshow([ptCloud_filtered2.Location(maxIdx,:);ptCloud_filtered2.Location(pairPointsIdx(maxIdx),:)],'MarkerSize',1000);
    %pcshow(lar_points,'MarkerSize',1000);
    setplotlayout("preprocessing",params)
    title(strcat('D_GT =',{' '},num2str(data_list{i,2}),'mm','    D_l_seg =',{' '},num2str(D_l_seg*1000),'mm'),'Interpreter','none','Color', 'k')
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-03-l_seg_det.png')), '-dpng', '-r400');
    end
    if save_fitted_pc
        writematrix([pts_l_seg + apple_min_corner - [0 0.005 -0.005],VertexColour],fullfile(output_fitted_shapes,strcat(data_list{i,1}(1:end-4),'_lar_seg_pc.txt')));
    end
    %plot LS
    pts_LS2 = pts_LS + point_Cloud_shift - pointCloud_center;
    sphere_colour = [0.52 0.07 0.07];
    sphere_marker = 'square';
    figure;
    pcshow(ptCloud2,'MarkerSize',50);
    hold on
    VertexColour=repmat(sphere_colour,size(pts_LS2,1),1);
    patch('Faces',F3,'Vertices',pts_LS2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','flat','EdgeColor','black','EdgeAlpha',0.7,'FaceAlpha',0.6,'FaceVertexCData',VertexColour);
    title(strcat('D_GT =',{' '},num2str(data_list{i,2}),'mm','    D_LS =',{' '},num2str(D_LS*1000),'mm'),'Interpreter','none','Color', 'k')
    setplotlayout("preprocessing",params)
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-04-LS_det.png')), '-dpng', '-r400');
    end
    if save_fitted_pc
        writematrix([pts_LS + apple_min_corner - [0 0.005 -0.005],VertexColour],fullfile(output_fitted_shapes,strcat(data_list{i,1}(1:end-4),'_LS_pc.txt')));
    end

    %plot MSAC
    pts_MSAC2 = pts_MSAC + point_Cloud_shift - pointCloud_center;
    sphere_colour = [0.74 0.54 0.06];
    sphere_marker = 'o';
    figure;
    pcshow(ptCloud2,'MarkerSize',50);
    hold on
    VertexColour=repmat(sphere_colour,size(pts_MSAC2,1),1);
    patch('Faces',F3,'Vertices',pts_MSAC2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','flat','EdgeColor','black','EdgeAlpha',0.7,'FaceAlpha',0.6,'FaceVertexCData',VertexColour);
    title(strcat('D_GT =',{' '},num2str(data_list{i,2}),'mm','    D_MSAC =',{' '},num2str(D_MSAC*1000),'mm'),'Interpreter','none','Color', 'k')
    setplotlayout("preprocessing",params)
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-05-MSAC_det.png')), '-dpng', '-r400');
    end
    if save_fitted_pc
        writematrix([pts_MSAC + apple_min_corner - [0 0.005 -0.005],VertexColour],fullfile(output_fitted_shapes,strcat(data_list{i,1}(1:end-4),'_MSAC_pc.txt')));
    end

    %plot TM
    pts_TM2 = pts_TM + point_Cloud_shift - pointCloud_center;
    sphere_colour = [0.4 0.57 0.17];
    sphere_marker = '^';
    F_TM = boundary(pts_TM2);
    figure;
    pcshow(ptCloud2,'MarkerSize',50);
    hold on
    VertexColour=repmat(sphere_colour,size(pts_TM2,1),1);
    patch('Faces',F_TM,'Vertices',pts_TM2,...
        'LineWidth',0.5,'FaceLighting','phong',...
        'BackFaceLighting','unlit',...
        'AmbientStrength',0.3,'DiffuseStrength',0.6,...
        'SpecularExponent',10,'SpecularStrength',0.9,...
        'Tag','Icosphere','FaceColor','flat','EdgeColor','black','EdgeAlpha',0.7,'FaceAlpha',0.6,'FaceVertexCData',VertexColour);
    title(strcat('D_GT =',{' '},num2str(data_list{i,2}),'mm','    D_TM =',{' '},num2str(D_TM*1000),'mm'),'Interpreter','none','Color', 'k')
    setplotlayout("preprocessing",params)
    if save_results
        print(fullfile(output_folder,strcat(num2str(i,'%02u'),'-',plot_description,'-06-TM_det.png')), '-dpng', '-r400');
    end
    if save_fitted_pc
        writematrix([pts_TM + apple_min_corner - [0 0.005 -0.005],VertexColour],fullfile(output_fitted_shapes,strcat(data_list{i,1}(1:end-4),'_TM_pc.txt')));
    end

% w = waitforbuttonpress;
% close all
% end



















