%symbolic math parts for generating the correct rot2map matrix
syms rmapx rmapy rmapz rmapxyz roll pitch yaw;

rmapp = [1 0 0; 0 cos(pitch) -sin(pitch); 0 sin(pitch) cos(pitch)]
rmapr = [cos(roll) 0 sin(roll); 0 1 0; -sin(roll) 0 cos(roll)]
rmapy = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1]
rmaprpy = rmapy * rmapp * rmapr 



syms rbx rby rbz rbxyz BSx BSy BSz;

rbx = [1 0 0; 0 cos(BSx) -sin(BSx); 0 sin(BSx) cos(BSx)]
rby = [cos(BSy) 0 sin(BSy); 0 1 0; -sin(BSy) 0 cos(BSy)]
rbz = [cos(BSz) -sin(BSz) 0; sin(BSz) cos(BSz) 0; 0 0 1]
rbxyz = rbz * rby * rbx 