# -- coding: mbcs --
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__

import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import math
import os
# all values are in meter, kg and seconds
# m1, m2 and m3 are about first cassion depth
# s1 and s2 are about second cassion or skirt diameter
# A, B, C and D are about second cassion or skirt depth
# model_names = ['m1_s1_C','m2_s1_C','m2_s1_A','m2_s1_B','m2_s1_D','m2_s2_C','m2_s2_A','m2_s2_B','m2_s2_D','m3_s1_C'] # name all the models here # 'm2_s0_Z'
model_names = ['m2_s1_C','m2_s1_A','m2_s1_B','m2_s1_D']
for model_name in model_names:
    if model_name in mdb.models.keys():
        del mdb.models[model_name] 

#Returning value of l1, l2, d2 based on the parameters (m1-m3,s1-s2, A-D), whatever Given part in the paper, we should assign a variable to the constant values
def parameters_model(i):
    if('m1' in i):
        l_1 = -0.12
    elif('m3' in i):
        l_1 = -0.36
    else:
        l_1 = -0.24
    if('s1' in i):
        d_2 = -0.03
    elif('s2' in i):
        d_2 = -0.05
    else:
        d_2 = 0
    if('A' in i):
        l_2 = 0
    elif('B' in i):
        l_2 = -0.03
    elif('C' in i):
        l_2 = -0.06
    elif('D' in i):
        l_2 = -0.09
    else:
        l_2 = 0.0
    return l_1, d_2, l_2

for model_name in model_names:
    # creating model from the list-model_names     
    mdb.Model(name=model_name, modelType=STANDARD_EXPLICIT)
    # assign parameter values from model names
    l_1, d_2, l_2 = parameters_model(model_name)
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    # SKETCH###
    # revolution
    # sketch for internal cassion
    diameter_cassion, thickness, thickness_lid = -0.06, -0.002, 0.01  #These values are constant in all the models
    s = mdb.models[model_name].ConstrainedSketch(name='int_cassion_rvl', sheetSize=1.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.rectangle(point1=(diameter_cassion+thickness, 0.0), point2=(diameter_cassion, l_1))   
    s.ConstructionLine(point1=(0.0, -0.5), point2=(0.0, 0.5))
    s.FixedConstraint(entity=g[2]) 
    
    # Check if the skirt is there or not
    if(l_2 == 0):
        # this line does not mean anything.
        lid_radius = diameter_cassion+d_2+thickness
        #print('not to be modeled')
    else:
        lid_radius = diameter_cassion+d_2+2*thickness
        # sketch for skirt
        s = mdb.models[model_name].ConstrainedSketch(name='skirt_rvl', sheetSize=1.0)
        g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
        s.rectangle(point1=(diameter_cassion+d_2+2*thickness, 0.0), point2=(diameter_cassion+d_2+thickness, l_2))
        s.ConstructionLine(point1=(0.0, -0.5), point2=(0.0, 0.5))
        s.FixedConstraint(entity=g[2])
        
    
    # sketch for lid
    s = mdb.models[model_name].ConstrainedSketch(name='int_cassion_lid_rvl', sheetSize=1.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.rectangle(point1=(lid_radius, thickness_lid), point2=(0.0, 0.0))
    s.ConstructionLine(point1=(0.0, -0.5), point2=(0.0, 0.5))
    s.FixedConstraint(entity=g[2])

    # sketch for rod
    s = mdb.models[model_name].ConstrainedSketch(name='rod_rvl', sheetSize=1.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.rectangle(point1=(-0.005, 0.41), point2=(0.0, thickness_lid)) # length of rod
    s.ConstructionLine(point1=(0.0, -0.5), point2=(0.0, 0.5))
    s.FixedConstraint(entity=g[2])
    
    # extrusion 
    # sketch for sand     
    s = mdb.models[model_name].ConstrainedSketch(name='sand_xtr', sheetSize=1.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.rectangle(point1=(-0.25, 0.25), point2=(0.75, -0.75))
    
    # sketch for gravel
    s = mdb.models[model_name].ConstrainedSketch(name='gravel_xtr', sheetSize=1.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.rectangle(point1=(-0.25, 0.25), point2=(0.75, -0.75))

    
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    # PARTS###
    # create parts from revolution
    for i in mdb.models[model_name].sketches.keys():
        if('_rvl' in i):
            mdb.models[model_name].Part(dimensionality=THREE_D, name=i, type= DEFORMABLE_BODY)
            mdb.models[model_name].parts[i].BaseSolidRevolve(angle=360.0, flipRevolveDirection=OFF, sketch= mdb.models[model_name].sketches[i])
       

    # create parts from extrusion 
    extrude_sketches = ['sand_xtr', 'gravel_xtr']
    extrude_depth = [0.6-thickness_lid, 0.1]  # change according to the model
    for i in range(len(extrude_sketches)):
        p = mdb.models[model_name].Part(name=extrude_sketches[i], dimensionality=THREE_D, type=DEFORMABLE_BODY)
        p.BaseSolidExtrude(sketch=mdb.models[model_name].sketches[extrude_sketches[i]], depth=extrude_depth[i])

#############################################################################################################################################
#############################################################################################################################################
    # PARTITIONING PARTS BEFORE ASSEMBALY 
    all_prt = mdb.models[model_name].parts.keys()
    always_prt = ['int_cassion_lid_rvl', 'rod_rvl'] 
    temp_anl_parts = list(set(all_prt) - set(always_prt))
    fine_parts = list(set(all_prt) - set(['sand_xtr', 'gravel_xtr']))
    # print(temp_anl_parts)
    # partitioning the annular parts
    print('tst1')
    for i in temp_anl_parts:
        if('_rvl' in i):
            #   Partition 1
            p = mdb.models[model_name].parts[i]
            c = p.cells
            pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
            e, v, d = p.edges, p.vertices, p.datums
            p.PartitionCellByPlaneNormalToEdge(edge=e[2], point=v[0], cells=pickedCells)
            #   Partition 2
            pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
            e, v, d = p.edges, p.vertices, p.datums
            p.PartitionCellByPlaneNormalToEdge(edge=e[13], cells=pickedCells, point=p.InterestingPoint(edge=e[13], rule=MIDDLE))
    print('tst2')
    # partitioning the solid parts
    for i in always_prt:
        #   Partition 1
        p = mdb.models[model_name].parts[i]
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
        e, v, d = p.edges, p.vertices, p.datums
        p.PartitionCellByPlaneNormalToEdge(edge=e[2], point=v[1], cells=pickedCells)
        #   Partition 2
        pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
        e, v, d = p.edges, p.vertices, p.datums
        p.PartitionCellByPlaneNormalToEdge(edge=e[4], cells=pickedCells,  point=p.InterestingPoint(edge=e[4], rule=MIDDLE))

    print('tst3')
    # lid partition
    p = mdb.models[model_name].parts['int_cassion_lid_rvl']
    f, e, d = p.faces, p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=f[2], sketchUpEdge=e[17], sketchPlaneSide=SIDE1, origin=(-0.039895, 0.01, 0.039895))
    s = mdb.models[model_name].ConstrainedSketch(name='rod_part', sheetSize=0.266, gridSpacing=0.006, transform=t)
    g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    
    s.CircleByCenterPerimeter(center=(0.039895, -0.039895), point1=(0.044895,-0.039895))
    f = p.faces
    pickedFaces = f.getSequenceFromMask(mask=('[#8484 ]', ), )
    e1, d2 = p.edges, p.datums
    p.PartitionFaceBySketch(sketchUpEdge=e1[17], faces=pickedFaces, sketch=s)
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#f ]', ), )
    e, d = p.edges, p.datums
    pickedEdges =(e[0], e[4], e[7], e[9])
    p.PartitionCellByExtrudeEdge(line=d[1], cells=pickedCells, edges=pickedEdges, sense=FORWARD)

    # parition rod at each eccentricity; use create datum planes and partition using datum plane

    p = mdb.models[model_name].parts['rod_rvl']
    f = p.faces
    p.DatumPlaneByOffset(plane=f[13], flip=SIDE2, offset=0.12, isDependent=False)
    p.DatumPlaneByOffset(plane=f[13], flip=SIDE2, offset=0.18, isDependent=False)
    p.DatumPlaneByOffset(plane=f[13], flip=SIDE2, offset=0.24, isDependent=False)
    p.DatumPlaneByOffset(plane=f[13], flip=SIDE2, offset=0.30, isDependent=False)

    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#f ]', ), )
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[4], cells=pickedCells)
    
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#78 ]', ), )
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[5], cells=pickedCells)
    
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#78 ]', ), )
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[6], cells=pickedCells)
    
    c = p.cells
    pickedCells = c.getSequenceFromMask(mask=('[#78 ]', ), )
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[7], cells=pickedCells)

    # create the loading points on the rod
    v = p.vertices
    verts = v.getSequenceFromMask(mask=('[#400000 ]', ), )
    p.Set(vertices=verts, name='e1')

    v = p.vertices
    verts = v.getSequenceFromMask(mask=('[#80000 ]', ), )
    p.Set(vertices=verts, name='e2')
    
    v = p.vertices
    verts = v.getSequenceFromMask(mask=('[#400 ]', ), )
    p.Set(vertices=verts, name='e3')
    
    v = p.vertices
    verts = v.getSequenceFromMask(mask=('[#10 ]', ), )
    p.Set(vertices=verts, name='e4')

    print('tst4')
    # partition the sand at levels for cassion and skirt
    all_prt = mdb.models[model_name].parts.keys()
    skirt_sum = [int('skirt' in i) for i in all_prt]
  
    if(sum(skirt_sum)!= 0): #if there is skirt
        # Layer partition for cassion and skirt
        p = mdb.models[model_name].parts['sand_xtr']
        f = p.faces
        p.DatumPlaneByOffset(plane=f[5], flip=SIDE2, offset=-1*l_2, isDependent=False)
        p.DatumPlaneByOffset(plane=f[5], flip=SIDE2, offset=-1*l_1, isDependent=False)
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
        d = p.datums
        p.PartitionCellByDatumPlane(datumPlane=d[3], cells=pickedCells)
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#2 ]', ), )
        d = p.datums
        p.PartitionCellByDatumPlane(datumPlane=d[2], cells=pickedCells)
        # cake cutting partition
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#7 ]', ), )
        e, v, d = p.edges, p.vertices, p.datums
        p.PartitionCellByPlanePointNormal(normal=e[25], cells=pickedCells, point=p.InterestingPoint(edge=e[25], rule=MIDDLE))
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#3f ]', ), )
        e, v, d = p.edges, p.vertices, p.datums
        p.PartitionCellByPlanePointNormal(normal=e[43], cells=pickedCells, point=p.InterestingPoint(edge=e[43], rule=MIDDLE))

    else: #if there is no skirt
        # Layer partition for cassion
        p = mdb.models[model_name].parts['sand_xtr']
        f = p.faces
        p.DatumPlaneByOffset(plane=f[5], flip=SIDE2, offset=-1*l_1, isDependent=False)
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#1 ]', ), )
        d = p.datums
        p.PartitionCellByDatumPlane(datumPlane=d[2], cells=pickedCells)
        # cake cutting partition
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#3 ]', ), )
        e, v, d1 = p.edges, p.vertices, p.datums
        p.PartitionCellByPlanePointNormal(normal=e[15], cells=pickedCells, point=p.InterestingPoint(edge=e[15], rule=MIDDLE))
        c = p.cells
        pickedCells = c.getSequenceFromMask(mask=('[#f ]', ), )
        e1, v1, d = p.edges, p.vertices, p.datums
        p.PartitionCellByPlanePointNormal(normal=e1[28], cells=pickedCells, point=p.InterestingPoint(edge=e1[28], rule=MIDDLE))
    print('tst5')
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    #ASSEMBLY        
    # Adding parts to assembly
    a = mdb.models[model_name].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    for i in mdb.models[model_name].parts.keys():
        p = mdb.models[model_name].parts[i]
        a.Instance(name=i+'-1', part=p, dependent=ON)

    # Rotate and fix the extruded parts
    a.rotate(instanceList=('sand_xtr-1','gravel_xtr-1'), axisPoint=(0.75, 0.25, 0.0), axisDirection=(-1.0, 0.0, 0.0), angle=-90.0)
    a.translate(instanceList=('sand_xtr-1','gravel_xtr-1'), vector=(-0.25, -0.25, 0.5))  # change according to the model
    a.translate(instanceList=('gravel_xtr-1', ), vector=(0.0, thickness_lid-0.6, 0.0))  # change according to the model

    # merge the parts to create monolithic parts    
    all_inst = mdb.models[model_name].rootAssembly.instances.keys()
    sand_all, gravel_all = [i for i in all_inst if 'sand' in i], [i for i in all_inst if 'gravel' in i]
    steel_all_1= list(set(all_inst) - set(gravel_all) )
    steel_all = list(set(steel_all_1) - set(sand_all) )
    #a.InstanceFromBooleanMerge(name='sand_all', instances= tuple([a.instances[i] for i in sand_all]), keepIntersections=ON, originalInstances=DELETE, domain=GEOMETRY)
    a.InstanceFromBooleanMerge(name='cassion_all', instances= tuple([a.instances[i] for i in steel_all]), keepIntersections=ON, originalInstances=DELETE, domain=GEOMETRY)
    a.InstanceFromBooleanCut(name='sand_all', instanceToBeCut=a.instances['sand_xtr-1'], cuttingInstances=(a.instances['cassion_all-1'], ), originalInstances=SUPPRESS)
    a.features['cassion_all-1'].resume()
    del a.features['sand_xtr-1']
    print('tst6')
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    #PROPERTIES
    el_mat = {'steel':[(7850.0, ), (210000000000.0, 0.3)], 'sand':[(1800.0, ), (10000000.0, 0.45)], 'gravel':[(2000.0, ), (80000000.0, 0.3)]}  
    for i,j in el_mat.items():
        #soil material
        mdb.models[model_name].Material(name=i)
        mdb.models[model_name].materials[i].Density(table=(j[0], ))
        mdb.models[model_name].materials[i].Elastic(table=(j[1], ))
    plast_mat = ['sand', 'gravel']
    phi_sand, phi_gravel, coh_all = 31, 40, 500 
    fric = [phi_sand, phi_gravel] # cohesion in pascals and phi in degrees
    permblt = [0.00001, 0.01] # permeability in m/sec
    void_r = [0.7, 0.7] # Void ratio in m/sec
    for i in range(len(plast_mat)):
        mdb.models[model_name].materials[plast_mat[i]].Permeability(inertialDragCoefficient= 0.142887, specificWeight=9810.0, table=((permblt[i], void_r[i]), ))
        mdb.models[model_name].materials[plast_mat[i]].MohrCoulombPlasticity(table=((fric[i], fric[i]-30), ))
        mdb.models[model_name].materials[plast_mat[i]].mohrCoulombPlasticity.MohrCoulombHardening( table=((coh_all, 0.0), ))
        mdb.models[model_name].materials[plast_mat[i]].mohrCoulombPlasticity.TensionCutOff(temperatureDependency=OFF, dependencies=0, table=(( round(coh_all*(1/math.tan(math.radians(fric[i]))),2), 0.0), ))
        
    # Assigning materials to the sections
    for i in mdb.models[model_name].materials.keys():
            mdb.models[model_name].HomogeneousSolidSection(name=i+'_section',material=i, thickness=None)
            
    # Section assignment to the parts
    all_prt = mdb.models[model_name].parts.keys()
    skirt_sum = [int('skirt' in i) for i in all_prt]
    print('tst7')
    if(sum(skirt_sum)!= 0): #if there is skirt
        p = mdb.models[model_name].parts['sand_all']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#ffffff ]', ), )
        region = p.Set(cells=cells, name='main')
        p.SectionAssignment(region=region, sectionName='sand_section', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        p = mdb.models[model_name].parts['cassion_all']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#ffffffff #f ]', ), )
        region = p.Set(cells=cells, name='main')
        p.SectionAssignment(region=region, sectionName='steel_section', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
        print('tst8')
    else: #if there is no skirt
        p = mdb.models[model_name].parts['sand_all']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#fff ]', ), )
        region = p.Set(cells=cells, name='main')
        p.SectionAssignment(region=region, sectionName='sand_section', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)

        p = mdb.models[model_name].parts['cassion_all']
        c = p.cells
        cells = c.getSequenceFromMask(mask=('[#ffffffff ]', ), )
        region = p.Set(cells=cells, name='main')
        p.SectionAssignment(region=region, sectionName='steel_section', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    print('tst9')
    p = mdb.models[model_name].parts['gravel_xtr']
    c = p.cells
    cells = c.getSequenceFromMask(mask=('[#1 ]', ), )
    region = p.Set(cells=cells, name='main')
    p.SectionAssignment(region=region, sectionName='gravel_section', offset=0.0, offsetType=MIDDLE_SURFACE, offsetField='', thicknessAssignment=FROM_SECTION)
    print('tst10')
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    #MESH  
    # assigning mesh to parts 

    elemType1 = mesh.ElemType(elemCode=C3D8P, elemLibrary=STANDARD)
    elemType2 = mesh.ElemType(elemCode=C3D6P, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=C3D4P, elemLibrary=STANDARD)

    main_dict =  {'sand_all':0.025, 'cassion_all':0.025, 'gravel_xtr':0.05}

    for i,j in main_dict.items():    
        
        p = mdb.models[model_name].parts[i]
        if(('sand' in i) or ('gravel' in i)):
            p.setElementType(regions=(p.cells.getSequenceFromMask(mask=('[#1 ]', ), ), ), elemTypes=(elemType1, elemType2, elemType3))
        p.seedPart(size=j, deviationFactor=0.1, minSizeFactor=0.1)
        p.generateMesh()
    print('tst11')
    # regenerating mesh for assemably
    a = mdb.models[model_name].rootAssembly
    a.regenerate()
    
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    #STEP
    # creating step for implicit dynamic loading

    mdb.models[model_name].GeostaticStep(name='geostatic', previous='Initial', timeIncrementationMethod=AUTOMATIC, initialInc=0.1, minInc=1e-05, maxInc=0.1, utol=1e-05, nlgeom=ON)
    mdb.models[model_name].SoilsStep(name='soils', previous='geostatic', timePeriod=30050.0, maxNumInc=1000, initialInc=0.1, minInc=0.0001, maxInc=100, end=None, utol=10000.0, cetol=None)
    mdb.models[model_name].fieldOutputRequests['F-Output-1'].setValues(frequency=LAST_INCREMENT)
    mdb.models[model_name].fieldOutputRequests['F-Output-1'].setValuesInStep(stepName='soils', timeInterval=600)
    mdb.models[model_name].historyOutputRequests['H-Output-1'].setValues(frequency=LAST_INCREMENT)
    mdb.models[model_name].historyOutputRequests['H-Output-1'].setValuesInStep(stepName='soils', timeInterval=600)
    

    # creating bottom set for fixed boundry
    p = mdb.models[model_name].parts['gravel_xtr']
    faces = p.faces.getSequenceFromMask(mask=('[#10 ]', ), )
    p.Set(faces=faces, name='bottom')
    mdb.models[model_name].EncastreBC(createStepName='Initial', localCsys=None, name='fixed', region= a.instances['gravel_xtr-1'].sets['bottom'])

    # creating roller set for roller boundry and other surfaces lower_sand

    all_prt = mdb.models[model_name].parts.keys()
    skirt_sum = [int('skirt' in i) for i in all_prt]
    print('tst12')
    if(sum(skirt_sum)!= 0): #if there is skirt
        p = mdb.models[model_name].parts['sand_all']
        faces = p.faces.getSequenceFromMask(mask=('[#0:2 #7094e000 #7fbce]', ), )  
        p.Set(faces=faces, name='roller')
    else: #if there is no skirt
        p = mdb.models[model_name].parts['sand_all']
        faces = p.faces.getSequenceFromMask(mask=('[#80000000 #7f9d049]', ), )  
        p.Set(faces=faces, name='roller')

    print('tst13')      
    p = mdb.models[model_name].parts['gravel_xtr']
    faces = p.faces.getSequenceFromMask(mask=('[#f ]', ), )
    p.Set(faces=faces, name='roller')

    print('tst14')    

    # creating roller set for roller boundry 
    # creating ROLLER set from boolean
    a.SetByBoolean(name='roller', sets=( a.allInstances['gravel_xtr-1'].sets['roller'], a.allInstances['sand_all-1'].sets['roller'], ))
    # roller BC
    mdb.models[model_name].DisplacementBC(name='roller', createStepName='Initial', region=a.sets['roller'], u1=SET, u2=UNSET, u3=SET, ur1=UNSET, 
    ur2=SET, ur3=UNSET, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)


    #creating PWP suraface at top of upper_sand TO BE DONE

    #Sand top PWP
    
    all_prt = mdb.models[model_name].parts.keys()
    skirt_sum = [int('skirt' in i) for i in all_prt]
    p = mdb.models[model_name].parts['sand_all']
    f = p.faces
    if(sum(skirt_sum)!= 0): #if there is skirt
        
        faces = f.getSequenceFromMask(mask=('[#0 #80000000 #2000024 ]', ), )
        p.Set(faces=faces, name='pwp_top')
        a = mdb.models[model_name].rootAssembly
        region = a.instances['sand_all-1'].sets['pwp_top']
        mdb.models[model_name].PorePressure(name='pwp_top_skirt', region=region, porePressure1=196.2, distributionType=UNIFORM, variation=CONSTANT_RATIO)
    
    else: #if there is no skirt
       
        faces = f.getSequenceFromMask(mask=('[#5000000 #220 ]', ), )
        p.Set(faces=faces, name='pwp_top')
        a = mdb.models[model_name].rootAssembly
        region = a.instances['sand_all-1'].sets['pwp_top']
        mdb.models[model_name].PorePressure(name='pwp_top_skirt', region=region, porePressure1=196.2, distributionType=UNIFORM,  variation=CONSTANT_RATIO)
   
    # creating contact friction sand_steel

    mdb.models[model_name].ContactProperty('sand_steel')
    mdb.models[model_name].interactionProperties['sand_steel'].TangentialBehavior(formulation=PENALTY, directionality=ISOTROPIC, slipRateDependency=OFF, pressureDependency=OFF, 
    temperatureDependency=OFF, dependencies=0, table=(( round(math.tan(math.radians(phi_sand))/3, 3), ), ), shearStressLimit=None, maximumElasticSlip=FRACTION, fraction=0.005, elasticSlipStiffness=None)
    mdb.models[model_name].interactionProperties['sand_steel'].NormalBehavior( pressureOverclosure=HARD, allowSeparation=ON, constraintEnforcementMethod=DEFAULT)
            
    # creating general contact for all surface
#############################################################################################################################################
#############################################################################################################################################

    # create tie conditions and general contacts (use if else) where skirts are not there #
    mdb.models[model_name].ContactStd(name='sand_steel', createStepName='Initial')
    mdb.models[model_name].interactions['sand_steel'].includedPairs.setValuesInStep(stepName='Initial', useAllstar=ON)
    mdb.models[model_name].interactions['sand_steel'].contactPropertyAssignments.appendInStep(stepName='Initial', assignments=((GLOBAL, SELF, 'sand_steel'),))

#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    #LOAD

    # creating the Predefined fields
    # Void ratio

    all_inst = mdb.models[model_name].rootAssembly.instances.keys()
    for i in all_inst:
        if(('sand' in i) or ('gravel' in i)):
            region = a.instances[i].sets['main']
            mdb.models[model_name].VoidsRatio(name='vr_'+i, region=region, voidsRatio1=0.7, distributionType=UNIFORM, variation=CONSTANT_RATIO)

    #Creating geostatic stress sand and gravel    
    a = mdb.models[model_name].rootAssembly
    #region = a.instances['sand_all-1'].sets['main']
    #mdb.models[model_name].GeostaticStress(name='geostatic_stress_sand', region=region, stressMag1=0.0, vCoord1=0.0, stressMag2=13734.0, vCoord2=-0.7, lateralCoeff1=1-round(math.sin(math.radians(phi_sand))), lateralCoeff2=None)     
    #region1 = a.instances['gravel_xtr-1'].sets['main']
    #mdb.models[model_name].GeostaticStress(name='geostatic_stress_gravel', region=region1, stressMag1=13734.0, vCoord1=-0.7, stressMag2=15696.0, vCoord2=-0.8,lateralCoeff1=1-round(math.sin(math.radians(phi_gravel))), lateralCoeff2=None)
  

    # creating gravity load      
    mdb.models[model_name].Gravity(name='gravity', createStepName='geostatic', comp2=-9.8, distributionType=UNIFORM, field='')

    # apply point load at the partitioned edge ; maybe create a loop here

    #region = a.instances['rod_rvl-1'].sets['e2']
    #mdb.models[model_name].TabularAmplitude(name='load_amp', timeSpan=STEP, smooth=SOLVER_DEFAULT, data=tuple([(i*0.1, i*0.01) for i in range(101)]) )

    load_increment = 2.1  # Load increment in N
    time_increment = 600  # Time increment in seconds
    time_points, loads = [0], [0] # initialte the time and load columns
    current_time, current_load = 1, 2.1   # initialte the current_time and current_load
    for i in range(100): # fill the columns using the literature
        time_points.append(current_time)
        loads.append(current_load)
        if(len(time_points)%2 == 0):
            current_time += time_increment
        else:
            current_time += 1
            current_load += load_increment
    #list(zip(time_points, loads))

    # Create the amplitude
    mdb.models[model_name].TabularAmplitude(name='load_amp', timeSpan=STEP, smooth=SOLVER_DEFAULT, data=list(zip(time_points, loads)))

    a = mdb.models[model_name].rootAssembly
    region = a.instances['cassion_all-1'].sets['e2']
    mdb.models[model_name].ConcentratedForce(name='point_load', createStepName='soils', region=region, cf3=1.0, amplitude='load_amp', distributionType=UNIFORM, field='', localCsys=None)
    
#############################################################################################################################################
#############################################################################################################################################
#############################################################################################################################################
    #JOB 
    # creating job and submission
    a = mdb.models[model_name].rootAssembly
    a.regenerate()
    mdb.Job(name=model_name+'_job', model=model_name, description='', type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=50, 
        memoryUnits=PERCENTAGE, getMemoryFromAnalysis=False, explicitPrecision=DOUBLE, nodalOutputPrecision=FULL, echoPrint=OFF, activateLoadBalancing=False, 
        modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=4, 
        numGPUs=0, numDomains=4)

    #INPUT File Generation
    mdb.jobs[model_name+'_job'].writeInput(consistencyChecking=OFF)
