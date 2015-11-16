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

#include "LocalDislocationDensityCalculator.h"

#include "SIMPLib/Common/Constants.h"
#include "SIMPLib/Math/GeometryMath.h"
#include "SIMPLib/FilterParameters/AbstractFilterParametersWriter.h"

#include "SIMPLib/FilterParameters/FloatVec3FilterParameter.h"
#include "SIMPLib/FilterParameters/DataContainerSelectionFilterParameter.h"
#include "SIMPLib/FilterParameters/DataArraySelectionFilterParameter.h"
#include "SIMPLib/FilterParameters/StringFilterParameter.h"
#include "SIMPLib/FilterParameters/AbstractFilterParametersReader.h"
#include "SIMPLib/FilterParameters/SeparatorFilterParameter.h"
#include "SIMPLib/Math/SIMPLibMath.h"
#include "SIMPLib/Geometry/ImageGeom.h"
#include "SIMPLib/Geometry/EdgeGeom.h"


// Include the MOC generated file for this class
#include "moc_LocalDislocationDensityCalculator.cpp"



// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
LocalDislocationDensityCalculator::LocalDislocationDensityCalculator() :
  AbstractFilter(),
  m_EdgeDataContainerName(DREAM3D::Defaults::DataContainerName),
  m_BurgersVectorsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::EdgeAttributeMatrixName, DREAM3D::EdgeData::BurgersVectors),
  m_SlipPlaneNormalsArrayPath(DREAM3D::Defaults::DataContainerName, DREAM3D::Defaults::EdgeAttributeMatrixName, DREAM3D::EdgeData::SlipPlaneNormals),
  m_OutputDataContainerName(DREAM3D::Defaults::NewDataContainerName),
  m_OutputAttributeMatrixName(DREAM3D::Defaults::CellAttributeMatrixName),
  m_OutputArrayName("DislocationLineDensity"),
  m_DominantSystemArrayName("DominantSystem"),
  m_OutputArray(NULL),
  m_DominantSystemArray(NULL),
  m_DomainBounds(NULL),
  m_BurgersVectors(NULL),
  m_SlipPlaneNormals(NULL)
{
  m_CellSize.x = 2.0;
  m_CellSize.y = 2.0;
  m_CellSize.z = 2.0;

  setupFilterParameters();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
LocalDislocationDensityCalculator::~LocalDislocationDensityCalculator()
{
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LocalDislocationDensityCalculator::setupFilterParameters()
{
  FilterParameterVector parameters;

  parameters.push_back(FloatVec3FilterParameter::New("Cell Size (Microns)", "CellSize", getCellSize(), FilterParameter::Parameter));
// parameters.push_back(SeparatorFilterParameter::New("", FilterParameter::Uncategorized));
  {
    DataContainerSelectionFilterParameter::RequirementType req;
    parameters.push_back(DataContainerSelectionFilterParameter::New("Edge DataContainer", "EdgeDataContainerName", getEdgeDataContainerName(), FilterParameter::RequiredArray, req));
  }
  {
    DataArraySelectionFilterParameter::RequirementType req;
    parameters.push_back(DataArraySelectionFilterParameter::New("Burgers Vectors Array", "BurgersVectorsArrayPath", getBurgersVectorsArrayPath(), FilterParameter::RequiredArray, req));
  }
  {
    DataArraySelectionFilterParameter::RequirementType req;
    parameters.push_back(DataArraySelectionFilterParameter::New("Slip Plane Normals Array", "SlipPlaneNormalsArrayPath", getSlipPlaneNormalsArrayPath(), FilterParameter::RequiredArray, req));
  }
//  parameters.push_back(SeparatorFilterParameter::New("", FilterParameter::Uncategorized));
  parameters.push_back(StringFilterParameter::New("Volume Data Container", "OutputDataContainerName", getOutputDataContainerName(), FilterParameter::CreatedArray));
  parameters.push_back(StringFilterParameter::New("Cell AttributeMatrix", "OutputAttributeMatrixName", getOutputAttributeMatrixName(), FilterParameter::CreatedArray));
  parameters.push_back(StringFilterParameter::New("Dislocation Line Density Array Name", "OutputArrayName", getOutputArrayName(), FilterParameter::CreatedArray));
  parameters.push_back(StringFilterParameter::New("Dominant System Array Name", "DominantSystemArrayName", getDominantSystemArrayName(), FilterParameter::CreatedArray));
  setFilterParameters(parameters);
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LocalDislocationDensityCalculator::readFilterParameters(AbstractFilterParametersReader* reader, int index)
{
  reader->openFilterGroup(this, index);
  setEdgeDataContainerName( reader->readString( "EdgeDataContainerName", getEdgeDataContainerName() ) );
  setSlipPlaneNormalsArrayPath(reader->readDataArrayPath("SlipPlaneNormalsArrayPath", getSlipPlaneNormalsArrayPath()));
  setBurgersVectorsArrayPath(reader->readDataArrayPath("BurgersVectorsArrayPath", getBurgersVectorsArrayPath()));
  setOutputDataContainerName(reader->readString("OutputDataContainerName", getOutputDataContainerName()));
  setOutputAttributeMatrixName( reader->readString( "OutputAttributeMatrixName", getOutputAttributeMatrixName() ) );
  setOutputArrayName(reader->readString("OutputArrayName", getOutputArrayName()));
  setDominantSystemArrayName(reader->readString("DominantSystemArrayName", getDominantSystemArrayName()));
  setCellSize(reader->readFloatVec3("CellSize", getCellSize()));
  reader->closeFilterGroup();
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int LocalDislocationDensityCalculator::writeFilterParameters(AbstractFilterParametersWriter* writer, int index)
{
  writer->openFilterGroup(this, index);
  SIMPL_FILTER_WRITE_PARAMETER(EdgeDataContainerName)
  SIMPL_FILTER_WRITE_PARAMETER(OutputDataContainerName)
  SIMPL_FILTER_WRITE_PARAMETER(OutputAttributeMatrixName)
  SIMPL_FILTER_WRITE_PARAMETER(OutputArrayName)
  SIMPL_FILTER_WRITE_PARAMETER(DominantSystemArrayName)
  SIMPL_FILTER_WRITE_PARAMETER(SlipPlaneNormalsArrayPath)
  SIMPL_FILTER_WRITE_PARAMETER(BurgersVectorsArrayPath)
  SIMPL_FILTER_WRITE_PARAMETER(CellSize)
  writer->closeFilterGroup();
  return ++index; // we want to return the next index that was just written to
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LocalDislocationDensityCalculator::updateCellInstancePointers()
{
  setErrorCondition(0);

  if (NULL != m_OutputArrayPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {
    m_OutputArray = m_OutputArrayPtr.lock()->getPointer(0);
  } /* Now assign the raw pointer to data from the DataArray<T> object */
  if (NULL != m_DominantSystemArrayPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {
    m_DominantSystemArray = m_DominantSystemArrayPtr.lock()->getPointer(0);
  } /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LocalDislocationDensityCalculator::dataCheck()
{
  DataArrayPath tempPath;
  setErrorCondition(0);

  // First sanity check the inputs and output names. All must be filled in

  if(getOutputDataContainerName().isEmpty() == true)
  {
    QString ss = QObject::tr("The output DataContainer name is empty. Please assign a name for the created DataContainer");
    setErrorCondition(-11001);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  if(getOutputAttributeMatrixName().isEmpty() == true)
  {
    QString ss = QObject::tr("The output AttributeMatrix name is empty. Please assign a name for the created AttributeMatrix");
    setErrorCondition(-11002);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  if(getOutputArrayName().isEmpty() == true)
  {
    QString ss = QObject::tr("The output array name is empty. Please assign a name for the created array");
    setErrorCondition(-11003);
    notifyErrorMessage(getHumanLabel(), ss, getErrorCondition());
  }

  // we can not go any further until all of the names are set.
  if(getErrorCondition() < 0) { return; }

  // Next check the existing DataContainer/AttributeMatrix
  DataContainer::Pointer m = getDataContainerArray()->getPrereqDataContainer<AbstractFilter>(this, getEdgeDataContainerName());
  if(getErrorCondition() < 0) { return; }

  EdgeGeom::Pointer edges = m->getPrereqGeometry<EdgeGeom, AbstractFilter>(this);
  if(getErrorCondition() < 0) { return; }

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
  //We MUST also have the domain bounds of the edge data container
  QVector<size_t> dims(1, 6);
  tempPath.update(getEdgeDataContainerName(), "_MetaData", "DomainBounds");
  m_DomainBoundsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this, tempPath, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_DomainBoundsPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  { m_DomainBounds = m_DomainBoundsPtr.lock()->getPointer(0); } /* Now assign the raw pointer to data from the DataArray<T> object */

  // Create a new DataContainer
  DataContainer::Pointer m2 = getDataContainerArray()->createNonPrereqDataContainer<AbstractFilter>(this, getOutputDataContainerName());
  if(getErrorCondition() < 0) { return; }

  //Create the voxel geometry to hold the local densities
  ImageGeom::Pointer image = ImageGeom::CreateGeometry(DREAM3D::Geometry::ImageGeometry);
  m2->setGeometry(image);

  //Create the cell attrMat in the new data container
  QVector<size_t> tDims(3, 0);
  AttributeMatrix::Pointer newCellAttrMat = m2->createNonPrereqAttributeMatrix<AbstractFilter>(this, getOutputAttributeMatrixName(), tDims, DREAM3D::AttributeMatrixType::Cell);
  if(getErrorCondition() < 0) { return; }

  //Get the name and create the array in the new data attrMat
  dims[0] = 3;
  m_BurgersVectorsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this, getBurgersVectorsArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_BurgersVectorsPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {m_BurgersVectors = m_BurgersVectorsPtr.lock()->getPointer(0);} /* Now assign the raw pointer to data from the DataArray<T> object */
  m_SlipPlaneNormalsPtr = getDataContainerArray()->getPrereqArrayFromPath<DataArray<float>, AbstractFilter>(this, getSlipPlaneNormalsArrayPath(), dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_SlipPlaneNormalsPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {m_SlipPlaneNormals = m_SlipPlaneNormalsPtr.lock()->getPointer(0);} /* Now assign the raw pointer to data from the DataArray<T> object */
  dims[0] = 1;
  tempPath.update(getOutputDataContainerName(), getOutputAttributeMatrixName(), getOutputArrayName());
  m_OutputArrayPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<float>, AbstractFilter, float>(this, tempPath, 0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_OutputArrayPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {m_OutputArray = m_OutputArrayPtr.lock()->getPointer(0);} /* Now assign the raw pointer to data from the DataArray<T> object */
  dims[0] = 1;
  tempPath.update(getOutputDataContainerName(), getOutputAttributeMatrixName(), getDominantSystemArrayName());
  m_DominantSystemArrayPtr = getDataContainerArray()->createNonPrereqArrayFromPath<DataArray<float>, AbstractFilter, float>(this, tempPath, 0, dims); /* Assigns the shared_ptr<> to an instance variable that is a weak_ptr<> */
  if (NULL != m_DominantSystemArrayPtr.lock().get()) /* Validate the Weak Pointer wraps a non-NULL pointer to a DataArray<T> object */
  {m_DominantSystemArray = m_DominantSystemArrayPtr.lock()->getPointer(0);} /* Now assign the raw pointer to data from the DataArray<T> object */
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
void LocalDislocationDensityCalculator::preflight()
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
void LocalDislocationDensityCalculator::execute()
{
  QString ss;
  setErrorCondition(0);
  dataCheck();
  if(getErrorCondition() < 0) { return; }

  DataContainer::Pointer edc = getDataContainerArray()->getDataContainer(getEdgeDataContainerName());
  DataContainer::Pointer vdc = getDataContainerArray()->getDataContainer(getOutputDataContainerName());
  AttributeMatrix::Pointer cellAttrMat = vdc->getAttributeMatrix(getOutputAttributeMatrixName());
  EdgeGeom::Pointer edgeGeom = edc->getGeometryAs<EdgeGeom>();

  float* nodes = edgeGeom->getVertexPointer(0);
  int64_t* edge = edgeGeom->getEdgePointer(0);
  size_t numEdges = edgeGeom->getNumberOfEdges();

  float xMin = m_DomainBounds[0];
  float yMin = m_DomainBounds[1];
  float zMin = m_DomainBounds[2];
  float xMax = m_DomainBounds[3];
  float yMax = m_DomainBounds[4];
  float zMax = m_DomainBounds[5];

  FloatVec3_t halfCellSize;
  halfCellSize.x = (m_CellSize.x / 2.0);
  halfCellSize.y = (m_CellSize.y / 2.0);
  halfCellSize.z = (m_CellSize.z / 2.0);
  FloatVec3_t quarterCellSize;
  quarterCellSize.x = (m_CellSize.x / 4.0);
  quarterCellSize.y = (m_CellSize.y / 4.0);
  quarterCellSize.z = (m_CellSize.z / 4.0);

  vdc->getGeometryAs<ImageGeom>()->setOrigin(xMin, yMin, zMin);
  size_t dcDims[3];
  dcDims[0] = size_t((xMax - xMin) / halfCellSize.x);
  dcDims[1] = size_t((yMax - yMin) / halfCellSize.y);
  dcDims[2] = size_t((zMax - zMin) / halfCellSize.z);
  vdc->getGeometryAs<ImageGeom>()->setDimensions(dcDims[0], dcDims[1], dcDims[2]);
  vdc->getGeometryAs<ImageGeom>()->setResolution(m_CellSize.x / 2.0, m_CellSize.y / 2.0, m_CellSize.z / 2.0);

  QVector<size_t> tDims(3, 0);
  tDims[0] = dcDims[0];
  tDims[1] = dcDims[1];
  tDims[2] = dcDims[2];
  cellAttrMat->resizeAttributeArrays(tDims);
  updateCellInstancePointers();

  //Create a temporary array to hold the individual slip system lengths in order to determine dominant system
  FloatArrayType::Pointer m_IndividualSystemLengthsPtr = FloatArrayType::CreateArray(12 * m_OutputArrayPtr.lock()->getNumberOfTuples(), "INDIVIDUAL_SYSTEM_LENGTHS_INTERNAL_USE_ONLY");
  float* m_IndividualSystemLengths = m_IndividualSystemLengthsPtr->getPointer(0);

  float point1[3], point2[3], corner1[3], corner2[3];
  size_t xCellMin, xCellMax;
  size_t yCellMin, yCellMax;
  size_t zCellMin, zCellMax;
  float x1, y1, z1, x2, y2, z2;
  size_t zStride, yStride, point;
  float length;
  int32_t system = 0;
  for(size_t i = 0; i < numEdges; i++)
  {
    point1[0] = nodes[3 * edge[2 * i + 0] + 0];
    point1[1] = nodes[3 * edge[2 * i + 0] + 1];
    point1[2] = nodes[3 * edge[2 * i + 0] + 2];
    point2[0] = nodes[3 * edge[2 * i + 1] + 0];
    point2[1] = nodes[3 * edge[2 * i + 1] + 1];
    point2[2] = nodes[3 * edge[2 * i + 1] + 2];
    x1 = (point1[0] - xMin);
    y1 = (point1[1] - yMin);
    z1 = (point1[2] - zMin);
    x2 = (point2[0] - xMin);
    y2 = (point2[1] - yMin);
    z2 = (point2[2] - zMin);
    if(x1 > x2) { xCellMin = size_t(x2 / quarterCellSize.x), xCellMax = size_t(x1 / quarterCellSize.x); }
    else { xCellMin = size_t(x1 / quarterCellSize.x), xCellMax = size_t(x2 / quarterCellSize.x); }
    if(y1 > y2) { yCellMin = size_t(y2 / quarterCellSize.y), yCellMax = size_t(y1 / quarterCellSize.y); }
    else { yCellMin = size_t(y1 / quarterCellSize.y), yCellMax = size_t(y2 / quarterCellSize.y); }
    if(z1 > z2) { zCellMin = size_t(z2 / quarterCellSize.z), zCellMax = size_t(z1 / quarterCellSize.z); }
    else { zCellMin = size_t(z1 / quarterCellSize.z), zCellMax = size_t(z2 / quarterCellSize.z); }
    xCellMin = (xCellMin - 1) / 2;
    yCellMin = (yCellMin - 1) / 2;
    zCellMin = (zCellMin - 1) / 2;
    xCellMax = ((xCellMax - 1) / 2) + 1;
    yCellMax = ((yCellMax - 1) / 2) + 1;
    zCellMax = ((zCellMax - 1) / 2) + 1;
    if (xCellMax >= tDims[0]) { xCellMax = tDims[0] - 1; }
    if (yCellMax >= tDims[1]) { yCellMax = tDims[1] - 1; }
    if (zCellMax >= tDims[2]) { zCellMax = tDims[2] - 1; }
    for (size_t j = zCellMin; j <= zCellMax; j++)
    {
      zStride = j * tDims[0] * tDims[1];
      corner1[2] = (j * halfCellSize.z) - halfCellSize.z + quarterCellSize.z + zMin;
      corner2[2] = (j * halfCellSize.z) + halfCellSize.z + quarterCellSize.z + zMin;
      for(size_t k = yCellMin; k <= yCellMax; k++)
      {
        yStride = k * tDims[0];
        corner1[1] = (k * halfCellSize.y) - halfCellSize.y + quarterCellSize.y + yMin;
        corner2[1] = (k * halfCellSize.y) + halfCellSize.y + quarterCellSize.y + yMin;
        for(size_t l = xCellMin; l <= xCellMax; l++)
        {
          corner1[0] = (l * halfCellSize.x) - halfCellSize.x + quarterCellSize.x + xMin;
          corner2[0] = (l * halfCellSize.x) + halfCellSize.x + quarterCellSize.x + xMin;
          length = GeometryMath::LengthOfRayInBox(point1, point2, corner1, corner2);
          point = (zStride + yStride + l);
          m_OutputArray[point] += length;
          system = determine_slip_system(i);
          m_IndividualSystemLengths[12 * point + system] += length;
        }
      }
    }
  }

  float cellVolume = m_CellSize.x * m_CellSize.y * m_CellSize.z;
  for(size_t j = 0; j < tDims[2]; j++)
  {
    zStride = j * tDims[0] * tDims[1];
    for(size_t k = 0; k < tDims[1]; k++)
    {
      yStride = k * tDims[0];
      for(size_t l = 0; l < tDims[0]; l++)
      {
        point = (zStride + yStride + l);
        //take care of total density first before looping over all systems
        m_OutputArray[point] /= cellVolume;
        //convert to m/mm^3 from um/um^3
        m_OutputArray[point] *= 1.0E12f;
        float max = 0.0f;
        for (int iter = 0; iter < 12; iter++)
        {
          m_IndividualSystemLengths[12 * point + iter] /= cellVolume;
          //convert to m/mm^3 from um/um^3
          m_IndividualSystemLengths[12 * point + iter] *= 1.0E12f;
          if (m_IndividualSystemLengths[12 * point + iter] > max)
          {
            m_DominantSystemArray[point] = iter;
            max = m_IndividualSystemLengths[12 * point + iter];
          }
        }
      }
    }
  }

  notifyStatusMessage(getHumanLabel(), "Complete");
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
int LocalDislocationDensityCalculator::determine_slip_system(int edgeNum)
{
  float planeFam1, planeFam2, planeFam3, planeFam4;
  float slipDir1, slipDir2, slipDir3, slipDir4, slipDir5, slipDir6;

  float tol = 0.000001f;

  int system = 12;
  planeFam1 = m_SlipPlaneNormals[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 1] * SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 2] * SIMPLib::Constants::k_1OverRoot3;
  planeFam2 = m_SlipPlaneNormals[3 * edgeNum + 0] * -SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 1] * SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 2] * SIMPLib::Constants::k_1OverRoot3;
  planeFam3 = m_SlipPlaneNormals[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 1] * -SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 2] * SIMPLib::Constants::k_1OverRoot3;
  planeFam4 = m_SlipPlaneNormals[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 1] * SIMPLib::Constants::k_1OverRoot3 + m_SlipPlaneNormals[3 * edgeNum + 2] * -SIMPLib::Constants::k_1OverRoot3;
  slipDir1 = m_BurgersVectors[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 1] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 2] * 0.0;
  slipDir2 = m_BurgersVectors[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 1] * 0.0 + m_BurgersVectors[3 * edgeNum + 2] * SIMPLib::Constants::k_1OverRoot2;
  slipDir3 = m_BurgersVectors[3 * edgeNum + 0] * 0.0 + m_BurgersVectors[3 * edgeNum + 1] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 2] * SIMPLib::Constants::k_1OverRoot2;
  slipDir4 = m_BurgersVectors[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 1] * -SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 2] * 0.0;
  slipDir5 = m_BurgersVectors[3 * edgeNum + 0] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 1] * 0.0 + m_BurgersVectors[3 * edgeNum + 2] * -SIMPLib::Constants::k_1OverRoot2;
  slipDir6 = m_BurgersVectors[3 * edgeNum + 0] * 0.0 + m_BurgersVectors[3 * edgeNum + 1] * SIMPLib::Constants::k_1OverRoot2 + m_BurgersVectors[3 * edgeNum + 2] * -SIMPLib::Constants::k_1OverRoot2;
  if (fabs(fabs(planeFam1) - 1.0) < tol)
  {
    if (fabs(fabs(slipDir4) - 1.0) < tol) { system = 0; }
    if (fabs(fabs(slipDir5) - 1.0) < tol) { system = 1; }
    if (fabs(fabs(slipDir6) - 1.0) < tol) { system = 2; }
  }
  if (fabs(fabs(planeFam2) - 1.0) < tol)
  {
    if (fabs(fabs(slipDir1) - 1.0) < tol) { system = 3; }
    if (fabs(fabs(slipDir2) - 1.0) < tol) { system = 4; }
    if (fabs(fabs(slipDir6) - 1.0) < tol) { system = 5; }
  }
  if (fabs(fabs(planeFam3) - 1.0) < tol)
  {
    if (fabs(fabs(slipDir1) - 1.0) < tol) { system = 6; }
    if (fabs(fabs(slipDir3) - 1.0) < tol) { system = 7; }
    if (fabs(fabs(slipDir5) - 1.0) < tol) { system = 8; }
  }
  if (fabs(fabs(planeFam4) - 1.0) < tol)
  {
    if (fabs(fabs(slipDir2) - 1.0) < tol) { system = 9; }
    if (fabs(fabs(slipDir3) - 1.0) < tol) { system = 10; }
    if (fabs(fabs(slipDir4) - 1.0) < tol) { system = 11; }
  }
  return system;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
AbstractFilter::Pointer LocalDislocationDensityCalculator::newFilterInstance(bool copyFilterParameters)
{
  LocalDislocationDensityCalculator::Pointer filter = LocalDislocationDensityCalculator::New();
  if(true == copyFilterParameters)
  {
    copyFilterParameterInstanceVariables(filter.get());
  }
  return filter;
}

// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString LocalDislocationDensityCalculator::getCompiledLibraryName()
{ return DDDAnalysisToolboxConstants::DDDAnalysisToolboxBaseName; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString LocalDislocationDensityCalculator::getGroupName()
{ return DREAM3D::FilterGroups::Unsupported; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString LocalDislocationDensityCalculator::getSubGroupName()
{ return DREAM3D::FilterSubGroups::StatisticsFilters; }


// -----------------------------------------------------------------------------
//
// -----------------------------------------------------------------------------
const QString LocalDislocationDensityCalculator::getHumanLabel()
{ return "Calculate Local Dislocation Densities"; }

