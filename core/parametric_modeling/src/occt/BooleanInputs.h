#pragma once

#include <NCollection_List.hxx>
#include <TopoDS_Shape.hxx>

namespace modeling::occt {
// Two-shape Boolean constructors already Build(). Use empty constructors so
// options are applied before the one and only build, including fuzzy values.
template<class Operation>
void setBooleanInputs(Operation& operation, const TopoDS_Shape& argument, const TopoDS_Shape& tool) {
    NCollection_List<TopoDS_Shape> arguments, tools;
    arguments.Append(argument);
    tools.Append(tool);
    operation.SetArguments(arguments);
    operation.SetTools(tools);
}
}
