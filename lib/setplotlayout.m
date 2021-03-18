function setplotlayout(plot_type,params)
if plot_type=="preprocessing"
    camorbit(params(1),0,'data',[0 0 1]);
    camorbit(params(2),0,'data',[0 1 0]);
    camorbit(params(3),0,'data',[1 0 0]);
    set(gca,'Color','white')
    set(gca,'XLim',[0,params(4)])
    set(gca,'YLim',[0,params(5)])
    set(gca,'ZLim',[0,params(6)])
    set(gca,'XColor',[0.2,0.2,0.2])
    set(gca,'YColor',[0.2,0.2,0.2])
    set(gca,'ZColor',[0.2,0.2,0.2])
    set(gcf,'color','w');
    set(gcf,'Position',[110 110 400 300]);
    
%     set(gca,'XColor','white')
%     set(gca,'YColor','white')
%     set(gca,'ZColor','white')
end
end