modules = HebiLookup.newGroupFromFamily('*');
cmd = CommandStruct();
tic
while true
    x = sin(toc * 0.1);
    cmd.position = [0 0 x];
    modules.send(cmd);
    pause(0.01);
end