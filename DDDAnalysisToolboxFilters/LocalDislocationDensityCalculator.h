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

#ifndef _localdislocationdensitycalculator_h_
#define _localdislocationdensitycalculator_h_

#include <QtCore/QString>
#include <set>

#include "SIMPLib/Common/Constants.h"
#include "SIMPLib/Common/SIMPLibSetGetMacros.h"
#include "SIMPLib/DataArrays/IDataArray.h"
#include "SIMPLib/DataContainers/DataContainer.h"
#include "SIMPLib/FilterParameters/FloatVec3FilterParameter.h"
#include "SIMPLib/Filtering/AbstractFilter.h"
#include "SIMPLib/Geometry/MeshStructs.h"
#include "SIMPLib/SIMPLib.h"

#include "DDDAnalysisToolbox/DDDAnalysisToolboxConstants.h"

/**
 * @class LocalDislocationDensityCalculator LocalDislocationDensityCalculator.h /FilterCategoryFilters/LocalDislocationDensityCalculator.h
 * @brief
 * @author
 * @date
 * @version 1.0
 */
class LocalDislocationDensityCalculator : public AbstractFilter
{
    Q_OBJECT
  public:
    SIMPL_SHARED_POINTERS(LocalDislocationDensityCalculator)
    SIMPL_STATIC_NEW_MACRO(LocalDislocationDensityCalculator)
     SIMPL_TYPE_MACRO_SUPER_OVERRIDE(LocalDislocationDensityCalculator, AbstractFilter)

    virtual ~LocalDislocationDensityCalculator();
    SIMPL_FILTER_PARAMETER(QString, EdgeDataContainerName)
    Q_PROPERTY(QString EdgeDataContainerName READ getEdgeDataContainerName WRITE setEdgeDataContainerName)

    SIMPL_FILTER_PARAMETER(DataArrayPath, BurgersVectorsArrayPath)
    Q_PROPERTY(DataArrayPath BurgersVectorsArrayPath READ getBurgersVectorsArrayPath WRITE setBurgersVectorsArrayPath)

    SIMPL_FILTER_PARAMETER(DataArrayPath, SlipPlaneNormalsArrayPath)
    Q_PROPERTY(DataArrayPath SlipPlaneNormalsArrayPath READ getSlipPlaneNormalsArrayPath WRITE setSlipPlaneNormalsArrayPath)

    SIMPL_FILTER_PARAMETER(FloatVec3_t, CellSize)
    Q_PROPERTY(FloatVec3_t CellSize READ getCellSize WRITE setCellSize)

    // The user selects a new DataContainerName
    SIMPL_FILTER_PARAMETER(QString, OutputDataContainerName)
    Q_PROPERTY(QString OutputDataContainerName READ getOutputDataContainerName WRITE setOutputDataContainerName)
    // Name the new AttributeMatrix that will get created
    SIMPL_FILTER_PARAMETER(QString, OutputAttributeMatrixName)
    Q_PROPERTY(QString OutputAttributeMatrixName READ getOutputAttributeMatrixName WRITE setOutputAttributeMatrixName)

    // Give the created data array a name
    SIMPL_FILTER_PARAMETER(QString, OutputArrayName)
    Q_PROPERTY(QString OutputArrayName READ getOutputArrayName WRITE setOutputArrayName)

    SIMPL_FILTER_PARAMETER(QString, DominantSystemArrayName)
    Q_PROPERTY(QString DominantSystemArrayName READ getDominantSystemArrayName WRITE setDominantSystemArrayName)


    /**
     * @brief getCompiledLibraryName Reimplemented from @see AbstractFilter class
     */
    virtual const QString getCompiledLibraryName() const override;

    /**
     * @brief getBrandingString Returns the branding string for the filter, which is a tag
     * used to denote the filter's association with specific plugins
     * @return Branding string
    */
    virtual const QString getBrandingString() const override;

    /**
     * @brief getFilterVersion Returns a version string for this filter. Default
     * value is an empty string.
     * @return
     */
    virtual const QString getFilterVersion() const override;

    /**
     * @brief newFilterInstance Reimplemented from @see AbstractFilter class
     */
    virtual AbstractFilter::Pointer newFilterInstance(bool copyFilterParameters) const override;

    /**
     * @brief getGroupName Reimplemented from @see AbstractFilter class
     */
    virtual const QString getGroupName() const override;

    /**
     * @brief getSubGroupName Reimplemented from @see AbstractFilter class
     */
    virtual const QString getSubGroupName() const override;

    /**
     * @brief getUuid Return the unique identifier for this filter.
     * @return A QUuid object.
     */
    virtual const QUuid getUuid() override;

    /**
     * @brief getHumanLabel Reimplemented from @see AbstractFilter class
     */
    virtual const QString getHumanLabel() const override;

    /**
    * @brief This method will instantiate all the end user settable options/parameters
    * for this filter
    */
    virtual void setupFilterParameters() override;

    /**
    * @brief This method will read the options from a file
    * @param reader The reader that is used to read the options from a file
    */
    virtual void readFilterParameters(AbstractFilterParametersReader* reader, int index);

    /**
    * @brief Reimplemented from @see AbstractFilter class
    */
    virtual void execute() override;

    /**
    * @brief This function runs some sanity checks on the DataContainer and inputs
    * in an attempt to ensure the filter can process the inputs.
    */
    virtual void preflight() override;

  signals:
    void updateFilterParameters(AbstractFilter* filter);
    void parametersChanged();
    void preflightAboutToExecute();
    void preflightExecuted();

  protected:
    LocalDislocationDensityCalculator();

    /**
     * @brief dataCheck Checks for the appropriate parameter values and availability of arrays
     */
    void dataCheck();

    /**
     * @brief Initializes all the private instance variables.
     */
    void initialize();

    void updateCellInstancePointers();

    int determine_slip_system(int edgeNum);

  private:
    DEFINE_DATAARRAY_VARIABLE(float, OutputArray)
    DEFINE_DATAARRAY_VARIABLE(float, DominantSystemArray)
    DEFINE_DATAARRAY_VARIABLE(float, DomainBounds)
    DEFINE_DATAARRAY_VARIABLE(float, BurgersVectors)
    DEFINE_DATAARRAY_VARIABLE(float, SlipPlaneNormals)

    LocalDislocationDensityCalculator(const LocalDislocationDensityCalculator&) = delete; // Copy Constructor Not Implemented
    void operator=(const LocalDislocationDensityCalculator&) = delete;                    // Operator '=' Not Implemented
};

#endif /* LocalDislocationDensityCalculator_H_ */






