function [] = velo_diff(velo1,velo2)

    pc1=topointcloud(velo1);
    pc2=topointcloud(velo2);
    figure
    pcshowpair(pc1,pc2,'VerticalAxis','Y','VerticalAxisDir','Down')
    title('Difference Between Two Point Clouds')
    xlabel('X(m)')
    ylabel('Y(m)')
    zlabel('Z(m)')
end

