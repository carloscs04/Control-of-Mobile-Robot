function [v,w] = unicycle_saturation(wrmax,wlmax,v,w,r,d)
    
    if w>2*wrmax*r/d
        w=2*wrmax*r/d;
    end
    if w<-(2*wrmax*r/d)
        w=-2*wrmax*r/d;
    end
    
    if v>r*wrmax-d/2*w
        v=r*wrmax-d/2*w;
    end
    if v>r*wlmax+d/2*w
        v=r*wlmax+d/2*w;
    end
    
    if v<-(r*wrmax-d/2*w)
        v=-(r*wrmax-d/2*w);
    end
    if v<-(r*wlmax+d/2*w)
        v=-(r*wlmax+d/2*w);
    end
end

