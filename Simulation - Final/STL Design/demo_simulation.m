
robot = robot_arm_auto();
show(robot); axis equal


function robot = robot_arm_auto()
here = fileparts(mfilename('fullpath'));
meshPath = fullfile(here,'..','meshes','arm_auto');
robot = importrobot(fullfile(meshPath,'articulated_arm_auto.urdf'), ...
    "DataFormat","column","MeshPath",meshPath);
end

