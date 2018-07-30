#pragma once

#include "DDDAnalysisToolbox/DDDAnalysisToolboxPlugin.h"

class DDDAnalysisToolboxGuiPlugin : public DDDAnalysisToolboxPlugin
{
  Q_OBJECT
  Q_INTERFACES(ISIMPLibPlugin)
  Q_PLUGIN_METADATA(IID "net.bluequartz.dream3d.DDDAnalysisToolboxGuiPlugin")

public:
  DDDAnalysisToolboxGuiPlugin();
   ~DDDAnalysisToolboxGuiPlugin() override;
  
  /**
   * @brief Register all the filters with the FilterWidgetFactory
   */
  void registerFilterWidgets(FilterWidgetManager* fwm) override;
  

public:
  DDDAnalysisToolboxGuiPlugin(const DDDAnalysisToolboxGuiPlugin&) = delete;            // Copy Constructor Not Implemented
  DDDAnalysisToolboxGuiPlugin(DDDAnalysisToolboxGuiPlugin&&) = delete;                 // Move Constructor Not Implemented
  DDDAnalysisToolboxGuiPlugin& operator=(const DDDAnalysisToolboxGuiPlugin&) = delete; // Copy Assignment Not Implemented
  DDDAnalysisToolboxGuiPlugin& operator=(DDDAnalysisToolboxGuiPlugin&&) = delete;      // Move Assignment Not Implemented
};
