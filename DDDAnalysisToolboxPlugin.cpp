/*
 * Your License or Copyright Information can go here
 */

#include "DDDAnalysisToolboxPlugin.h"

#include <QtCore/QFile>
#include <QtCore/QFileInfo>

#include "SIMPLib/Filtering/FilterFactory.hpp"
#include "SIMPLib/Filtering/FilterManager.h"

#include "DDDAnalysisToolbox/DDDAnalysisToolboxConstants.h"
#include "DDDAnalysisToolbox/DDDAnalysisToolboxVersion.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DDDAnalysisToolboxPlugin::DDDAnalysisToolboxPlugin()
: m_Version(DDDAnalysisToolbox::Version::Package())
, // Initialize DDDAnalysisToolbox's Version Number Here
    m_CompatibilityVersion(DDDAnalysisToolbox::Version::Package())
, // Initialize DDDAnalysisToolbox's Compatibility Version Number Here
    m_Vendor("Open-Source")
, m_URL("http://www.github.com/dream3d/DDDAnalysisToolbox")
, m_Location("")
, // Initialize Company Location Here
    m_Copyright("")
, // Initialize DDDAnalysisToolbox's Copyright Here
    m_Filters(QList<QString>())
, // Initialize DDDAnalysisToolbox's List of Dependencies Here
    m_DidLoad(false)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
DDDAnalysisToolboxPlugin::~DDDAnalysisToolboxPlugin() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getPluginFileName()
{
  return DDDAnalysisToolboxConstants::DDDAnalysisToolboxPluginFile;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getPluginDisplayName()
{
  return DDDAnalysisToolboxConstants::DDDAnalysisToolboxPluginDisplayName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getPluginBaseName()
{
  return DDDAnalysisToolboxConstants::DDDAnalysisToolboxBaseName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getVersion()
{
  return m_Version;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getCompatibilityVersion()
{
  return m_CompatibilityVersion;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getVendor()
{
  return m_Vendor;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getURL()
{
  return m_URL;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getLocation()
{
  return m_Location;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getDescription()
{
  /* PLEASE UPDATE YOUR PLUGIN'S DESCRIPTION FILE.
  It is located at DDDAnalysisToolbox/Resources/DDDAnalysisToolbox/DDDAnalysisToolboxDescription.txt */

  QFile licenseFile(":/DDDAnalysisToolbox/DDDAnalysisToolboxDescription.txt");
  QFileInfo licenseFileInfo(licenseFile);
  QString text = "<<--Description was not read-->>";

  if(licenseFileInfo.exists())
  {
    if(licenseFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      QTextStream in(&licenseFile);
      text = in.readAll();
    }
  }
  return text;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getCopyright()
{
  return m_Copyright;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QString DDDAnalysisToolboxPlugin::getLicense()
{
  /* PLEASE UPDATE YOUR PLUGIN'S LICENSE FILE.
  It is located at DDDAnalysisToolbox/Resources/DDDAnalysisToolbox/DDDAnalysisToolboxLicense.txt */

  QFile licenseFile(":/DDDAnalysisToolbox/DDDAnalysisToolboxLicense.txt");
  QFileInfo licenseFileInfo(licenseFile);
  QString text = "<<--License was not read-->>";

  if(licenseFileInfo.exists())
  {
    if(licenseFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
      QTextStream in(&licenseFile);
      text = in.readAll();
    }
  }
  return text;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
QMap<QString, QString> DDDAnalysisToolboxPlugin::getThirdPartyLicenses()
{
  QMap<QString, QString> licenseMap;
  QList<QString> fileStrList;
  fileStrList.push_back(":/ThirdParty/HDF5.txt");

  fileStrList.push_back(":/ThirdParty/Qt.txt");
  fileStrList.push_back(":/ThirdParty/Qwt.txt");

  for(QList<QString>::iterator iter = fileStrList.begin(); iter != fileStrList.end(); iter++)
  {
    QFile file(*iter);
    QFileInfo licenseFileInfo(file);

    if(licenseFileInfo.exists())
    {
      if(file.open(QIODevice::ReadOnly | QIODevice::Text))
      {
        QTextStream in(&file);
        licenseMap.insert(licenseFileInfo.baseName(), in.readAll());
      }
    }
  }

  return licenseMap;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
bool DDDAnalysisToolboxPlugin::getDidLoad()
{
  return m_DidLoad;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DDDAnalysisToolboxPlugin::setDidLoad(bool didLoad)
{
  m_DidLoad = didLoad;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DDDAnalysisToolboxPlugin::setLocation(QString filePath)
{
  m_Location = filePath;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DDDAnalysisToolboxPlugin::writeSettings(QSettings& prefs)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DDDAnalysisToolboxPlugin::readSettings(QSettings& prefs)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void DDDAnalysisToolboxPlugin::registerFilterWidgets(FilterWidgetManager* fwm)
{
}

#include "DDDAnalysisToolboxFilters/RegisterKnownFilters.cpp"
