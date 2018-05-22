function u = quadCtrlAlloc(taustar)
global param

C =  [1,    1        ,   1           1 ;
      0, -param.QUAD.lprop,   0, param.QUAD.lprop;
      param.QUAD.lprop, 0, -param.QUAD.lprop, 0;
      -param.QUAD.cprop, param.QUAD.cprop, -param.QUAD.cprop, param.QUAD.cprop];
  
u = C\taustar;