function frame(O0,O,a)
    vec3(O0(:,1),O0(:,2),a,'r')
    vec3(O0(:,1),O0(:,3),a,'g')
    vec3(O0(:,1),O0(:,4),a,'b')
    label(strcat('$i_{',int2str(O),'}$'),O0(:,1)+O0(:,2),2,4)
    label(strcat('$j_{',int2str(O),'}$'),O0(:,1)+O0(:,3),2,4)
    label(strcat('$k_{',int2str(O),'}$'),O0(:,1)+O0(:,4),2,4)
end