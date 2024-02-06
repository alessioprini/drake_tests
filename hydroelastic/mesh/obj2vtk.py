#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import vtk

# Create a reader for the .stl file.
reader = vtk.vtkSTLReader()
reader.SetFileName(sys.argv[1])
reader.Update()

# Generate a tetrahedral mesh from the input points.
delaunay = vtk.vtkDelaunay3D()
delaunay.SetInputConnection(reader.GetOutputPort())
delaunay.Update()

# Convert the vtkPolyData to a vtkUnstructuredGrid.
appendFilter = vtk.vtkAppendFilter()
appendFilter.AddInputData(delaunay.GetOutput())
appendFilter.Update()

triangleFilter = vtk.vtkDataSetTriangleFilter()
triangleFilter.SetInputConnection(appendFilter.GetOutputPort())
triangleFilter.Update()

# Calcola il volume di ogni cella nella griglia.
quality = vtk.vtkCellQuality()
quality.SetInputData(reader.GetOutput())
quality.SetQualityMeasureToVolume()
quality.Update()

# Ottieni i volumi delle celle.
volumes = quality.GetOutput().GetCellData().GetArray("Quality")
print(quality.GetOutput().GetCellData())

# Stampa il volume di ogni cella.
for i in range(volumes.GetNumberOfTuples()):
    print("Volume of cell {}: {}".format(i, volumes.GetValue(i)))
# # Get the volumes of the cells.
# volumes = quality.GetOutput().GetCellData().GetArray("Quality")

# # Check if volumes is not None.
# if volumes is not None:
#     # Create a new grid that will contain only the cells with positive volume.
#     newGrid = vtk.vtkUnstructuredGrid()

#     # Iterate over the cells in the original grid.
#     for i in range(triangleFilter.GetOutput().GetNumberOfCells()):
#         # If the volume of the cell is positive, add it to the new grid.
#         if volumes.GetValue(i) > 0:
#             newGrid.InsertNextCell(triangleFilter.GetOutput().GetCell(i).GetCellType(), triangleFilter.GetOutput().GetCell(i).GetPointIds())
# else:
#     print("Failed to compute the volumes of the cells.")

# Create a writer for the .vtk file.
writer = vtk.vtkUnstructuredGridWriter()
writer.SetFileName(sys.argv[2])
writer.SetInputData(triangleFilter.GetOutput())
writer.Write()


# from __future__ import print_function
# import sys

# if(len(sys.argv) != 3):
#   print('Usage: ', sys.argv[0], 'input.obj output.vtk')
#   sys.exit()

# import vtk
# # read vtk version 4.2
# print(vtk.vtkVersion.GetVTKMajorVersion())
# reader = vtk.vtkOBJReader()
# reader.SetFileName(sys.argv[1])
# reader.Update()
# obj = reader.GetOutput()

# grid =  vtk.vtkUnstructuredGrid()
# grid.ShallowCopy(reader.GetOutput())

# cells = vtk.vtkCellDataToPointData().SetInputData(grid)


# # writer = vtk.vtkPolyDataWriter()
# writer = vtk.vtkUnstructuredGridWriter()
# writer.SetFileName(sys.argv[2])
# print(vtk.VTK_MAJOR_VERSION)
# if vtk.VTK_MAJOR_VERSION <= 5:
#     writer.SetInput(grid)
#     writer.SetInputData(cells)
# else:
#     writer.SetInputData(grid)
#     writer.SetInputData(cells)

# writer.Write()



# import sys
# import vtk

# # Create a reader for the .obj file.
# reader = vtk.vtkOBJReader()
# reader.SetFileName(sys.argv[1])
# reader.Update()

# # Generate a tetrahedral mesh from the input points.
# delaunay = vtk.vtkDelaunay3D()
# delaunay.SetInputConnection(reader.GetOutputPort())
# delaunay.Update()

# # Convert the vtkPolyData to a vtkUnstructuredGrid.
# appendFilter = vtk.vtkAppendFilter()
# appendFilter.AddInputData(delaunay.GetOutput())
# appendFilter.Update()

# triangleFilter = vtk.vtkDataSetTriangleFilter()
# triangleFilter.SetInputConnection(appendFilter.GetOutputPort())
# triangleFilter.Update()

# # Compute the volume of each cell in the grid.
# quality = vtk.vtkCellQuality()
# quality.SetInputData(triangleFilter.GetOutput())
# quality.SetQualityMeasureToVolume()
# quality.Update()

# print(quality.GetOutput().GetCellData())

# # Get the volumes of the cells.
# volumes = quality.GetOutput().GetCellData().GetArray("Quality")

# # Create a new grid that will contain only the cells with positive volume.
# newGrid = vtk.vtkUnstructuredGrid()

# # Iterate over the cells in the original grid.
# for i in range(triangleFilter.GetOutput().GetNumberOfCells()):
#     # If the volume of the cell is positive, add it to the new grid.
#     if volumes.GetValue(i) > 0:
#         newGrid.InsertNextCell(triangleFilter.GetOutput().GetCell(i).GetCellType(), triangleFilter.GetOutput().GetCell(i).GetPointIds())

# # Create a writer for the .vtk file.
# writer = vtk.vtkUnstructuredGridWriter()
# writer.SetFileName(sys.argv[2])
# writer.SetInputData(newGrid)
# writer.Write()