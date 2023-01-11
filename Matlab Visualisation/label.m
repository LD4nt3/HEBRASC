function label(l,p,h,v)
    switch h
        case 1
            h = 'left';
        case 2
            h = 'center';
        case 3
            h = 'rigth';
    end
    switch v
        case 1
            v = 'middle';
        case 2 
            v = 'top';
        case 3
            v = 'cap';
        case 4
            v = 'bottom';
        case 5
            v = 'baseline';
    end
    hold on
    if length(p) == 3
        text(p(1),p(2),p(3),l,'HorizontalAlignment',h,'VerticalAlignment',v)
    end
    hold off
end
            