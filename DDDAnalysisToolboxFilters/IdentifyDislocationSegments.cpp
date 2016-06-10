/* ============================================================================
 * Copyright (c) 2011 Michael A. Jackson (BlueQuartz Software)
 * Copyright (c) 2011 Dr. Michael A. Groeber (US Air Force Research Laboratories)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of Michael A. Groeber, Michael A. Jackson,
 * the US Air Force, BlueQuartz Software nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  This code was written under United States Air Force Contract number
 *                   FA8650-07-D-5800 and FA8650-10-D-5226
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

#include "IdentifyDislocationSegments.h"

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

#include "SIMPLib/Math/SIMPLibMath.h"
#include "SIMPLib/Math/GeometryMath.h"
#include "SIMPLib/Utilities/SIMPLibRandom.h"
#include "SIMPLib/FilterParameters/AbstractFilterParametersWriter.h"
#include "SIMPLib/FilterParameters/DataArraySelectionFilterParameter.h"
#include "SIMPLib/FilterParameters/StringFilterParameter.h"
#include "SIMPLib/FilterParameters/AbstractFilterParametersReader.h"
#include "SIMPLib/FilterParameters/SeparatorFilterParameter.h"
#include "SIMPLib/Geometry/EdgeGeom.h"

#include "DDDAnalysisToolbox/DDDAnalysisToolboxVersion.h"


// Include the MOC generated file for this class
#include "moc_IdentifyDislocationSegments.cpp"



// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
IdentifyDislocationSegments::IdentifyDislocationSegments() :
  AbstractFilter(),
  m_EdgeFeatureAttributeMatrixName(SIMPL::Defaults::EdgeFeatureAttributeMatrixName),
  m_BurgersVectorsArrayPath(SIMPL::Defaults::EdgeDataContainerName, SIMPL::Defaults::EdgeAttributeMatrixName, SIMPL::EdgeData::BurgersVectors),
  m_SlipPlaneNormalsArrayPath(SIMPL::Defaults::EdgeDataContainerName, SIMPL::Defaults::EdgeAttributeMatrixName, SIMPL::EdgeData::SlipPlaneNormals),
  m_DislocationIdsArrayName(SIMPL::EdgeData::DislocationIds),
  m_ActiveArrayName(SIMPL::FeatureData::Active),
  m_BurgersVectors(NULL),
  m_SlipPlaneNormals(NULL),
  m_DislocationIds(NULL),
  m_Active(NULL)
{
  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
IdentifyDislocationSegments::~IdentifyDislocationSegments()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(SeparatorFilterParameter::New("Edge Data", FilterParameter::RequiredArray));
  {
    DataArraySelectionFilterParameter::RequirementType req;
    parameters.push_back(DataArraySelectionFilterParameter::New("Burgers Vectors", "BurgersVectorsArrayPath", getBurgersVectorsArrayPath(), FilterParameter::RequiredArray, req));
  }
  {
    DataArraySelectionFilterParameter::RequirementType req;
    parameters.push_back(DataArraySelectionFilterParameter::New("Slip Plane Normals", "SlipPlaneNormalsArrayPath", getSlipPlaneNormalsArrayPath(), FilterParameter::RequiredArray, req));
  }

  parameters.push_back(SeparatorFilterParameter::New("Edge Data", FilterParameter::CreatedArray));
  parameters.push_back(StringFilterParameter::New("Dislocation Ids", "DislocationIdsArrayName", getDislocationIdsArrayName(), FilterParameter::CreatedArray));
  parameters.push_back(SeparatorFilterParameter::New("Edge Feature Data", FilterParameter::CreatedArray));
  parameters.push_back(StringFilterParameter::New("Edge Feature Attribute Matrix", "EdgeFeatureAttributeMatrixName", getEdgeFeatureAttributeMatrixName(), FilterParameter::CreatedArray));
  parameters.push_back(StringFilterParameter::New("Active", "ActiveArrayName", getActiveArrayName(), FilterParameter::CreatedArray));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setEdgeFeatureAttributeMatrixName(reader->readString("EdgeFeatureAttributeMatrixName", getEdgeFeatureAttributeMatrixName() ) );
  setActiveArrayName(reader->readString("ActiveArrayName", getActiveArrayName() ) );
  setDislocationIdsArrayName(reader->readString("DislocationIdsArrayName", getDislocationIdsArrayName() ) );
  setSlipPlaneNormalsArrayPath(reader->readDataArrayPath("SlipPlaneNormalsArrayPath", getSlipPlaneNormalsArrayPath() ) );
  setBurgersVectorsArrayPath(reader->readDataArrayPath("BurgersVectorsArrayPath", getBurgersVectorsArrayPath() ) );

  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int IdentifyDislocationSegments::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  SIMPL_FILTER_WRITE_PARAMETER(EdgeFeatureAttributeMatrixName)
  SIMPL_FILTER_WRITE_PARAMETER(ActiveArrayName)
  SIMPL_FILTER_WRITE_PARAMETER(DislocationIdsArrayName)
  SIMPL_FILTER_WRITE_PARAMETER(SlipPlaneNormalsArrayPath)
  SIMPL_FILTER_WRITE_PARAMETER(BurgersVectorsArrayPath)

  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::updateEdgeFeatureInstancePointers()
{
  setErrorCondition(0);

  if( NULL != m_ActivePtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_Active = m_ActivePtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::initialize()
{

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::dataCheck()
{
  DataArrayPath tempPath;
  setErrorCondition(0);

  // Next check the existing DataContainer/AttributeMatrix
  DataContainer::Pointer m = getDataContainerArray()->getPrereqDataContainer<AbstractFilter>(this, getBurgersVectorsArrayPath().getDataContainerName());
  if(getErrorCondition() < 0) { return; }
  QVector<size_t> tDims(1, 0);
  AttributeMatrix::Pointer edgeFeatureAttrMat = m->createNonPrereqAttributeMatrix<AbstractFilter>(this, getEdgeFeatureAttributeMatrixName(), tDims, SIMPL::AttributeMatrixType::EdgeFeature);
  if(getErrorCondition() < 0) { return; }

  EdgeGeom::Pointer edges = m->getPrereqGeometry<EdgeGeom, AbstractFilter>(this);
  // We MUST have Vertices defined.
  if(edges->getVertices().get() == NULL)
  {
    setErrorCondition(-384);
    notifyErrorMessage(getHumanLabel(), "DataContainer geometry missing Vertices", getErrorCondition());
  }
  // We MUST have Edges defined also.
  if(edges->getEdges().get() == NULL)
  {
    setErrorCondition(-384);
    notifyErrorMessage(getHumanLabel(), "DataContainer geometry missing Edges", getErrorCondition());
  }

  //Get the name and create the array in the new data attrMat
  QVector<size_t> dims(1, 3);
  m_BurgersVectorsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this,  getBurgersVectorsArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_BurgersVectorsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_BurgersVectors = m_BurgersVectorsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  m_SlipPlaneNormalsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this,  getSlipPlaneNormalsArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_SlipPlaneNormalsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_SlipPlaneNormals = m_SlipPlaneNormalsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  dims[0] = 1;
  tempPath.update(getBurgersVectorsArrayPath().getDataContainerName(), getBurgersVectorsArrayPath().getAttributeMatrixName(), getDislocationIdsArrayName() );
  m_DislocationIdsPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter, int32_t>(this, tempPath, 0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_DislocationIdsPtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_DislocationIds = m_DislocationIdsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  tempPath.update(getBurgersVectorsArrayPath().getDataContainerName(), getEdgeFeatureAttributeMatrixName(), getActiveArrayName() );
  m_ActivePtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<bool>, AbstractFilter, bool>(this, tempPath, true, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if( NULL != m_ActivePtr.lock().get() ) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_Active = m_ActivePtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::preflight()
{
  setInPreflight(true);
  emit preflightAboutToExecute();
  emit updateFilterParameters(this);
  dataCheck();
  emit preflightExecuted();
  setInPreflight(false);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::execute()
{
  QString ss;
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return; }

  DataContainer::Pointer m = getDataContainerArray()->getDataContainer(getBurgersVectorsArrayPath().getDataContainerName());
  AttributeMatrix::Pointer edgeFeatureAttrMat = m->getAttributeMatrix(getEdgeFeatureAttributeMatrixName());
  EdgeGeom::Pointer edgeGeom = m->getGeometryAs<EdgeGeom>();

  //float* vertex = edgeGeom->getVertexPointer(0);
  int64_t* edge = edgeGeom->getEdgePointer(0);
  size_t numEdges = edgeGeom->getNumberOfEdges();

  edgeGeom->findElementsContainingVert();

  DynamicListArray<uint16_t, int64_t>::Pointer edgesContainingVert = edgeGeom->getElementsContainingVert();

  int dnum = 0;
  qint32 size = 0;
  QVector<size_t> tDims(1, dnum);
  int64_t initialEdgesListSize = 100;
  QVector<int64_t> edgeslist(initialEdgesListSize, -1);
  float refBV[3], refSPN[3];
  float neighBV[3], neighSPN[3];
  float angleBV, angleSPN;
  float angleTol = 1.0 * SIMPLib::Constants::k_Pi / 180.0f;
  for(size_t i = 0; i < numEdges; i++)
  {
    size = 0;
    if(m_DislocationIds[i] == 0)
    {
      dnum++;
      m_DislocationIds[i] = dnum;
      edgeslist[size] = i;
      size++;
      tDims[0] = dnum + 1;
      edgeFeatureAttrMat->resizeAttributeArrays(tDims);
      updateEdgeFeatureInstancePointers();
      m_Active[dnum] = true;
      while(size > 0)
      {
        int64_t currentEdge = edgeslist[size - 1];
        refBV[0] = m_BurgersVectors[3 * currentEdge + 0];
        refBV[1] = m_BurgersVectors[3 * currentEdge + 1];
        refBV[2] = m_BurgersVectors[3 * currentEdge + 2];
        refSPN[0] = m_SlipPlaneNormals[3 * currentEdge + 0];
        refSPN[1] = m_SlipPlaneNormals[3 * currentEdge + 1];
        refSPN[2] = m_SlipPlaneNormals[3 * currentEdge + 2];
        size -= 1;
        for(int iter = 0; iter < 2; iter++)
        {
          uint16_t eCount = edgesContainingVert->getNumberOfElements(edge[2 * currentEdge + iter]);
          int64_t* data = edgesContainingVert->getElementListPointer(edge[2 * currentEdge + iter]);
          for(uint16_t j = 0; j < eCount; j++)
          {
            if(m_DislocationIds[data[j]] == 0)
            {
              neighBV[0] = m_BurgersVectors[3 * data[j] + 0];
              neighBV[1] = m_BurgersVectors[3 * data[j] + 1];
              neighBV[2] = m_BurgersVectors[3 * data[j] + 2];
              neighSPN[0] = m_SlipPlaneNormals[3 * data[j] + 0];
              neighSPN[1] = m_SlipPlaneNormals[3 * data[j] + 1];
              neighSPN[2] = m_SlipPlaneNormals[3 * data[j] + 2];
              angleBV = GeometryMath::AngleBetweenVectors(refBV, neighBV);
              angleSPN = GeometryMath::AngleBetweenVectors(refSPN, neighSPN);
              if((angleBV < angleTol || (SIMPLib::Constants::k_Pi - angleBV) < angleTol) && (angleSPN < angleTol || (SIMPLib::Constants::k_Pi - angleSPN) < angleTol))
              {
                m_DislocationIds[data[j]] = dnum;
                edgeslist[size] = data[j];
                size++;
                if(size >= edgeslist.size())
                {
                  size = edgeslist.size();
                  edgeslist.resize(size + initialEdgesListSize);
                  for(qint32 j = size; j < edgeslist.size(); ++j) { edgeslist[j] = -1; }
                }
              }
            }
          }
        }
      }
    }
  }

  // Generate all the numbers up front
  const int rangeMin = 1;
  const int rangeMax = dnum - 1;
  typedef boost::uniform_int<int> NumberDistribution;
  typedef boost::mt19937 RandomNumberGenerator;
  typedef boost::variate_generator<RandomNumberGenerator&, NumberDistribution> Generator;

  NumberDistribution distribution(rangeMin, rangeMax);
  RandomNumberGenerator generator;
  Generator numberGenerator(generator, distribution);
  generator.seed(static_cast<boost::uint32_t>( QDateTime::currentMSecsSinceEpoch() )); // seed with the current time

  DataArray<int32_t>::Pointer rndNumbers = DataArray<int32_t>::CreateArray(dnum, "New FeatureIds");
  int32_t* gid = rndNumbers->getPointer(0);
  gid[0] = 0;
  QSet<int32_t> featureIdSet;
  featureIdSet.insert(0);
  for(int32_t i = 1; i < dnum; ++i)
  {
    gid[i] = i; //numberGenerator();
    featureIdSet.insert(gid[i]);
  }

  qint32 r;
  qint32 temp;
  //--- Shuffle elements by randomly exchanging each with one other.
  for (qint32 i = 1; i < dnum; i++)
  {
    r = numberGenerator(); // Random remaining position.
    if (r >= dnum)
    {
      continue;
    }
    temp = gid[i];
    gid[i] = gid[r];
    gid[r] = temp;
  }

  // Now adjust all the Feature Id values for each Voxel
  for(size_t i = 0; i < numEdges; ++i)
  {
    m_DislocationIds[i] = gid[ m_DislocationIds[i] ];
  }

  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer IdentifyDislocationSegments::newFilterInstance(bool copyFilterParameters)
{
  IdentifyDislocationSegments::Pointer filter = IdentifyDislocationSegments::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getCompiledLibraryName()
{
  return DDDAnalysisToolboxConstants::DDDAnalysisToolboxBaseName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getBrandingString()
{
  return "DDDAnalysisToolbox";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getFilterVersion()
{
  QString version;
  QTextStream vStream(&version);
  vStream <<  DDDAnalysisToolbox::Version::Major() << "." << DDDAnalysisToolbox::Version::Minor() << "." << DDDAnalysisToolbox::Version::Patch();
  return version;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getGroupName()
{ return SIMPL::FilterGroups::Unsupported; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getSubGroupName()
{ return SIMPL::FilterSubGroups::FeatureIdentificationFilters; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getHumanLabel()
{ return "Identify Dislocation Segments"; }

