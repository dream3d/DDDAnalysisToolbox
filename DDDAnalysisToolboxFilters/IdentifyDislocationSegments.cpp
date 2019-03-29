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

#include <random>
#include <chrono>

#include "SIMPLib/FilterParameters/AbstractFilterParametersReader.h"
#include "SIMPLib/FilterParameters/DataArraySelectionFilterParameter.h"
#include "SIMPLib/FilterParameters/SeparatorFilterParameter.h"
#include "SIMPLib/FilterParameters/StringFilterParameter.h"
#include "SIMPLib/Geometry/EdgeGeom.h"
#include "SIMPLib/Math/GeometryMath.h"
#include "SIMPLib/Math/SIMPLibMath.h"
#include "SIMPLib/Math/SIMPLibRandom.h"

#include "DDDAnalysisToolbox/DDDAnalysisToolboxVersion.h"

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
IdentifyDislocationSegments::IdentifyDislocationSegments()
: m_EdgeFeatureAttributeMatrixName(SIMPL::Defaults::EdgeFeatureAttributeMatrixName)
, m_BurgersVectorsArrayPath(SIMPL::Defaults::EdgeDataContainerName, SIMPL::Defaults::EdgeAttributeMatrixName, SIMPL::EdgeData::BurgersVectors)
, m_SlipPlaneNormalsArrayPath(SIMPL::Defaults::EdgeDataContainerName, SIMPL::Defaults::EdgeAttributeMatrixName, SIMPL::EdgeData::SlipPlaneNormals)
, m_DislocationIdsArrayName(SIMPL::EdgeData::DislocationIds)
, m_ActiveArrayName(SIMPL::FeatureData::Active)
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
IdentifyDislocationSegments::~IdentifyDislocationSegments() = default;

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void IdentifyDislocationSegments::setupFilterParameters()
{
  FilterParameterVector parameters;
  parameters.push_back(SeparatorFilterParameter::New("Edge Data", FilterParameter::RequiredArray));
  {
    DataArraySelectionFilterParameter::RequirementType req;
    parameters.push_back(SIMPL_NEW_DA_SELECTION_FP("Burgers Vectors", BurgersVectorsArrayPath, FilterParameter::RequiredArray, IdentifyDislocationSegments, req));
  }
  {
    DataArraySelectionFilterParameter::RequirementType req;
    parameters.push_back(SIMPL_NEW_DA_SELECTION_FP("Slip Plane Normals", SlipPlaneNormalsArrayPath, FilterParameter::RequiredArray, IdentifyDislocationSegments, req));
  }

  parameters.push_back(SeparatorFilterParameter::New("Edge Data", FilterParameter::CreatedArray));
  parameters.push_back(SIMPL_NEW_STRING_FP("Dislocation Ids", DislocationIdsArrayName, FilterParameter::CreatedArray, IdentifyDislocationSegments));
  parameters.push_back(SeparatorFilterParameter::New("Edge Feature Data", FilterParameter::CreatedArray));
  parameters.push_back(SIMPL_NEW_STRING_FP("Edge Feature Attribute Matrix", EdgeFeatureAttributeMatrixName, FilterParameter::CreatedArray, IdentifyDislocationSegments));
  parameters.push_back(SIMPL_NEW_STRING_FP("Active", ActiveArrayName, FilterParameter::CreatedArray, IdentifyDislocationSegments));
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
void IdentifyDislocationSegments::updateEdgeFeatureInstancePointers()
{
  clearErrorCondition();
  clearWarningCondition();

  if(nullptr != m_ActivePtr.lock())                 /* Validate the Weak Pointer wraps a non-nullptr pointer to a DataArray<T> object */
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
  clearErrorCondition();
  clearWarningCondition();

  // Next check the existing DataContainer/AttributeMatrix
  DataContainer::Pointer m = getDataContainerArray()->getPrereqDataContainer(this, getBurgersVectorsArrayPath().getDataContainerName());
  if(getErrorCondition() < 0) { return; }
  QVector<size_t> tDims(1, 0);
  AttributeMatrix::Pointer edgeFeatureAttrMat = m->createNonPrereqAttributeMatrix(this, getEdgeFeatureAttributeMatrixName(), tDims, AttributeMatrix::Type::EdgeFeature);
  if(getErrorCondition() < 0) { return; }

  EdgeGeom::Pointer edges = m->getPrereqGeometry<EdgeGeom, AbstractFilter>(this);
  if(getErrorCondition() < 0) { return; }

  // We MUST have Vertices defined.
  if(edges->getVertices().get() == nullptr)
  {
    setErrorCondition(-384);
    notifyErrorMessage("DataContainer geometry missing Vertices", getErrorCondition());
  }
  // We MUST have Edges defined also.
  if(edges->getEdges().get() == nullptr)
  {
    setErrorCondition(-384);
    notifyErrorMessage("DataContainer geometry missing Edges", getErrorCondition());
  }

  //Get the name and create the array in the new data attrMat
  QVector<size_t> dims(1, 3);
  m_BurgersVectorsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this,  getBurgersVectorsArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if(nullptr != m_BurgersVectorsPtr.lock())                         /* Validate the Weak Pointer wraps a non-nullptr pointer to a DataArray<T> object */
  { m_BurgersVectors = m_BurgersVectorsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  m_SlipPlaneNormalsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this,  getSlipPlaneNormalsArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if(nullptr != m_SlipPlaneNormalsPtr.lock())                           /* Validate the Weak Pointer wraps a non-nullptr pointer to a DataArray<T> object */
  { m_SlipPlaneNormals = m_SlipPlaneNormalsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  dims[0] = 1;
  tempPath.update(getBurgersVectorsArrayPath().getDataContainerName(), getBurgersVectorsArrayPath().getAttributeMatrixName(), getDislocationIdsArrayName() );
  m_DislocationIdsPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<int32_t>, AbstractFilter, int32_t>(this, tempPath, 0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if(nullptr != m_DislocationIdsPtr.lock())                         /* Validate the Weak Pointer wraps a non-nullptr pointer to a DataArray<T> object */
  { m_DislocationIds = m_DislocationIdsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */
  tempPath.update(getBurgersVectorsArrayPath().getDataContainerName(), getEdgeFeatureAttributeMatrixName(), getActiveArrayName() );
  m_ActivePtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<bool>, AbstractFilter, bool>(this, tempPath, true, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if(nullptr != m_ActivePtr.lock())                 /* Validate the Weak Pointer wraps a non-nullptr pointer to a DataArray<T> object */
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
  clearErrorCondition();
  clearWarningCondition();
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

  std::random_device randomDevice;  //Will be used to obtain a seed for the random number engine
  std::mt19937_64 generator(randomDevice()); //Standard mersenne_twister_engine seeded with rd()
  std::mt19937_64::result_type seed = static_cast<std::mt19937_64::result_type>(std::chrono::steady_clock::now().time_since_epoch().count());
  generator.seed(seed);
  std::uniform_int_distribution<int32_t> distribution(rangeMin, rangeMax);

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
    r = distribution(generator); // Random remaining position.
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

}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer IdentifyDislocationSegments::newFilterInstance(bool copyFilterParameters) const
{
  IdentifyDislocationSegments::Pointer filter = IdentifyDislocationSegments::New();
  if(copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getCompiledLibraryName() const
{
  return DDDAnalysisToolboxConstants::DDDAnalysisToolboxBaseName;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getBrandingString() const
{
  return "DDDAnalysisToolbox";
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getFilterVersion() const
{
  QString version;
  QTextStream vStream(&version);
  vStream <<  DDDAnalysisToolbox::Version::Major() << "." << DDDAnalysisToolbox::Version::Minor() << "." << DDDAnalysisToolbox::Version::Patch();
  return version;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getGroupName() const
{ return SIMPL::FilterGroups::Unsupported; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QUuid IdentifyDislocationSegments::getUuid()
{
  return QUuid("{073798a1-1fb4-5e3c-81f6-e426f60e347a}");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getSubGroupName() const
{ return SIMPL::FilterSubGroups::FeatureIdentificationFilters; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString IdentifyDislocationSegments::getHumanLabel() const
{ return "Identify Dislocation Segments"; }

