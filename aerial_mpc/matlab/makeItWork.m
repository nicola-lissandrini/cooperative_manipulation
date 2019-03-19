function makeItWork (J)
    Jhand = matlabFunction (J);
    strJ = func2str (Jhand);
    strJ = regexprep (strJ,'l(\d)','params($1)');
    strJ = regexprep (strJ,'lbx','params(3)');
    strJ = regexprep (strJ,'lby','params(4)');
    strJ = regexprep (strJ,'lbz','params(5)');
    strJ = regexprep (strJ,'lox','params(6)');
    strJ = regexprep (strJ,'loy','params(7)');
    strJ = regexprep (strJ,'loz','params(8)');
    strJ = regexprep (strJ,'th(\d)','q($1)');
    strJ = regexprep (strJ,'xv','q(3)');
    strJ = regexprep (strJ,'yv','q(4)');
    strJ = regexprep (strJ,'zv','q(5)');
    strJ = regexprep (strJ,'psi','q(6)');
    
    disp ([strJ,';'])
 
end