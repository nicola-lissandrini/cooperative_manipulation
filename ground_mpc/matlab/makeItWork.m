function makeItWork (J)
    Jhand = matlabFunction (J);
    strJ = func2str (Jhand);
    strJ = regexprep (strJ,'l(\d)','params($1)');
    strJ = regexprep (strJ,'lox','params(4)');
    strJ = regexprep (strJ,'loy','params(5)');
    strJ = regexprep (strJ,'loz','params(6)');
    strJ = regexprep (strJ,'lbx','params(7)');
    strJ = regexprep (strJ,'lby','params(8)');
    strJ = regexprep (strJ,'lbz','params(9)');
    strJ = regexprep (strJ,'th(\d)','q($1)');
    strJ = regexprep (strJ,'xb','q(5)');
    strJ = regexprep (strJ,'yb','q(6)');
    strJ = regexprep (strJ,'psi','q(7)');
    
    disp (strJ)
 
end
