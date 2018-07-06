/*
 * Your License should go here
 */
#pragma once

#include <QtCore/QString>

/**
* @brief This namespace is used to define some Constants for the plugin itself.
*/
namespace DDDAnalysisToolboxConstants
{
  const QString DDDAnalysisToolboxPluginFile("DDDAnalysisToolboxPlugin");
  const QString DDDAnalysisToolboxPluginDisplayName("DDDAnalysisToolbox");
  const QString DDDAnalysisToolboxBaseName("DDDAnalysisToolbox");

  namespace FilterGroups
  {
    const QString DDDAnalyticsToolboxFilters("DDD Analytics");
  }
}

/**
* @brief Use this namespace to define any custom GUI widgets that collect FilterParameters
* for a filter. Do NOT define general reusable widgets here.
*/
namespace FilterParameterWidgetType
{

/*  const QString SomeCustomWidget("SomeCustomWidget"); */

}
