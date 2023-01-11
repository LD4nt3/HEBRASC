function figuresk(type,markersize,linewidth,fontsize)
set(gcf,'InvertHardCopy', 'off'); %setting 'grid color reset' off
switch type
    case 0
    case 1
        set(gca,'xticklabel',' ','yticklabel',' ','zticklabel',' ')
    case 2
        set(gcf,'color',[255 255 255]./255);
        set(gca,'color',[255 255 255]./255);
        set(gca,'xticklabel',' ','yticklabel',' ','zticklabel',' ')
end
set(findall(gcf,'Interpreter','tex'),'Interpreter','latex');
set(findall(gcf,'FontSize',10),'FontSize',fontsize);
set(findall(gcf,'Type','line'),'LineWidth',linewidth)
set(findall(gcf,'Type','line'),'MarkerSize',markersize)
grid minor
end