"""
IO functions for geoerefernecer. A to-do list:

:py:class:readRiegl A class for handling Riegl SDC/2DD/3DD files

:py:class:readNCOM A class for handling OXTS NCOM format trajectory files

:py:class:outputLAS Uses LASpy or PDAL to handle LAS writing

:py:class:outputPGPC outputs data as postgres-pointcloud table (requires PDAL and postgres-pointcloud)

:py:class:outputEntwine delivers an entwine index (requires PDAL)

:py:class:outputPotree delivers a Potree index

:py:class:outputPly delivers a Stanford PLY file

:py:class:outputNetCDF delivers a netCDF file with internal groups containing various data attributes as a self-contained package

:py:class:outputGeopackage delivers a Geopackage with points bundled in an sqlite table, formatted in postgres-pointcloud style 'patches'

#just getting started
Adam Steer
2017

"""
