Identify Dislocation Segments {#identifydislocationsegments}
======

## Group (Subgroup) ##
Unsupported (FeatureIdentification)

## Description ##
This Filter groups neighboring **Features** that have c-axes aligned within a user defined tolerance.  The algorithm for grouping the **Features** is analogous to the algorithm for segmenting the **Features** - only the average orientation of the **Features** are used instead of the orientations of the individual **Cells** and the criterion for grouping only considers the alignment of the c-axes.  The user can specify a tolerance for how closely aligned the c-axes must be for neighbor **Features** to be grouped.


NOTE: This filter is intended for use with *Hexagonal* materials.  While the c-axis is actually just referring to the <001> direction and thus will operate on any symmetry, the utility of grouping by <001> alignment is likely only important/useful in materials with anisotropy in that direction (like materials with *Hexagonal* symmetry).


## Parameters ##


## Required Arrays ##

| Type | Default Name | Description | Comment | Filters Known to Create Data |
|------|--------------|-------------|---------|-----|
| -- | Burgers Vectors | | | |
| -- | Slip Plane Normals | | | |

## Created Arrays ##

| Type | Default Name | Description | Comment |
|------|--------------|-------------|---------|
| Feature | Active | Boolean value specifying if the **Feature** is still in the sample (1 if the **Feature** is in the sample and 0 if it is not). | At the end of the filter, all **Features** will be "Active" as the "Inactive" **Features** will have been removed.  |
| -- | Edge Data | | |
| -- | Dislocation Ids | | |
| Feature | Edge Feature Data | | |



## License & Copyright ##

Please see the description file distributed with this plugin.

## DREAM3D Mailing Lists ##

If you need more help with a filter, please consider asking your question on the DREAM3D Users mailing list:
https://groups.google.com/forum/?hl=en#!forum/dream3d-users


