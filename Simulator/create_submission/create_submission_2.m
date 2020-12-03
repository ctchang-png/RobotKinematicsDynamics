% Create submission for Part 1 of the MATLAB simulator
function [] = create_submission_2()
    % The lab handin directory.
    currentDir = fileparts(mfilename('fullpath'));
    outputFile = fullfile(currentDir, 'handin-p2.tar.gz');

    if exist(outputFile,'file')
        delete(outputFile);
    end
    
    files = dir(fullfile(currentDir, '*.m'));
    
    filesRot = dir(fullfile(strcat(currentDir, '/Rotations'), '*.m'));
    filesPlanar = dir(fullfile(strcat(currentDir, '/PlanarKinematics'), '*.m'));
    files3D = dir(fullfile(strcat(currentDir, '/ThreeDKinematics'), '*.m'));
    filesVec = dir(fullfile(strcat(currentDir, '/Vectors'), '*.m'));
    filesJacobian = dir(fullfile(strcat(currentDir, '/Jacobian'), '*.m'));
    
    files = [files; filesRot; filesPlanar; files3D; filesVec; filesJacobian];
    
    files_string = {};
    for f = files'
        files_string{1, numel(files_string) + 1} = fullfile(f.folder, f.name);
    end
    tar(outputFile, files_string);
end
