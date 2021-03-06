
FCN_NAME = 'ground_solver';

if ( ispc == 0 )
    IFLAGS  = [ '-lrt -I./ground_MPC -I./ground_MPC/qpoases3/include  -I./ground_MPC/qpoases3/src  -I./ground_MPC/qpoases3/ ' ];
    CPPFLAGS  = [ IFLAGS, '-D__cpluplus -O -DLINUX CFLAGS="-fPIC -std=c99" -D__CODE_GENERATION__ ' ];
else
    IFLAGS  = [ '-I./ground_MPC -I./ground_MPC/qpoases3/include  -I./ground_MPC/qpoases3/src  -I./ground_MPC/qpoases3/ ' ];
    CPPFLAGS  = [ IFLAGS, '-D__cpluplus -O -DWIN32  -Dinline="" -D__CODE_GENERATION__ ' ];
end

OBJECTS = [ 'ground_MPC/qpoases3/src/QProblem.c ', ...
            'ground_MPC/qpoases3/src/QProblemB.c ', ...
            'ground_MPC/qpoases3/src/Bounds.c ', ...
            'ground_MPC/qpoases3/src/Matrices.c ', ...
            'ground_MPC/qpoases3/src/Constraints.c ', ...
            'ground_MPC/qpoases3/src/Indexlist.c ', ...
            'ground_MPC/qpoases3/src/Flipper.c ', ...
            'ground_MPC/qpoases3/src/Utils.c ', ...
            'ground_MPC/qpoases3/src/MessageHandling.c ', ...
            'ground_MPC/qpoases3/src/Options.c ', ...
            'ground_MPC/qpoases3/src/OQPinterface.c ', ...
            'ground_MPC/acado_integrator.c ',...
            'ground_MPC/acado_solver.c ',...
            'ground_MPC/acado_qpoases3_interface.c ' ];


eval( [ 'mex -v -output ', FCN_NAME, ' ', CPPFLAGS, ' ', FCN_NAME, '.c ', OBJECTS] );
disp( [ FCN_NAME, '.', eval('mexext'), ' successfully created!'] );

clear IFLAGS CPPFLAGS OBJECTS

