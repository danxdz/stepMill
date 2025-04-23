from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.Core.gp import gp_Pln, gp_Pnt, gp_Dir
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRepAdaptor import BRepAdaptor_Curve
from OCC.Core.GeomAbs import GeomAbs_Line, GeomAbs_Circle

def extract_profile_from_step(path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(path)
    if status != IFSelect_RetDone:
        raise ValueError("Failed to read STEP file")

    reader.TransferRoots()
    shape = reader.OneShape()

    plane = gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
    section = BRepAlgoAPI_Section(shape, plane, False)
    section.ComputePCurveOn1(True)
    section.Approximation(True)
    section.Build()

    segments = []
    explorer = TopExp_Explorer(section.Shape(), TopAbs_EDGE)
    while explorer.More():
        edge = explorer.Current()
        adaptor = BRepAdaptor_Curve(edge)
        ctype = adaptor.GetType()
        p1 = adaptor.Value(adaptor.FirstParameter())
        p2 = adaptor.Value(adaptor.LastParameter())

        x1, z1 = abs(p1.X()), p1.Z()
        x2, z2 = abs(p2.X()), p2.Z()

        if x1 < 1e-4 and x2 < 1e-4:
            explorer.Next()
            continue

        if ctype == GeomAbs_Line:
            segments.append(("LINE", (x1, z1), (x2, z2)))
        elif ctype == GeomAbs_Circle:
            radius = adaptor.Circle().Radius()
            segments.append(("ARC", (x1, z1), (x2, z2), radius))

        explorer.Next()

    return segments
