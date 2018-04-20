

#include "DDDAnalysisToolboxGuiPlugin.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DDDAnalysisToolboxGuiPlugin::DDDAnalysisToolboxGuiPlugin()
: DDDAnalysisToolboxPlugin()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DDDAnalysisToolboxGuiPlugin::~DDDAnalysisToolboxGuiPlugin() = default;

#include "DDDAnalysisToolbox/Gui/FilterParameterWidgets/RegisterKnownFilterParameterWidgets.cpp"
