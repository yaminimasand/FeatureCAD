#include <iostream>
#include <string>
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <gp_Pnt.hxx>
#include <cmath>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

using namespace std;

// Function to calculate the bounding sphere from the bounding box
void CalculateBoundingSphere(const Bnd_Box& box, gp_Pnt& center, Standard_Real& radius) {
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    // Calculate the center point
    center.SetCoord(
        (xMin + xMax) / 2.0,
        (yMin + yMax) / 2.0,
        (zMin + zMax) / 2.0
    );

    // Calculate the radius as half of the diagonal
    Standard_Real dx = xMax - xMin;
    Standard_Real dy = yMax - yMin;
    Standard_Real dz = zMax - zMin;
    radius = std::sqrt(dx * dx + dy * dy + dz * dz) / 2.0;
}

int main() {
    // Load STEP File
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile("./00000006_d4fe04f0f5f84b52bd4f10e4_step_001.step");
    if (status != IFSelect_RetDone) {
        cerr << "Error reading STEP file!" << endl;
        return 1;
    }
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    cout << "STEP file loaded successfully." << endl;

    // Maps for Counting Shapes
    TopTools_IndexedMapOfShape vertices, edges, faces;
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_FACE, faces);

    // Geometric Properties
    GProp_GProps volumeProps, surfaceProps;
    BRepGProp::VolumeProperties(shape, volumeProps);
    BRepGProp::SurfaceProperties(shape, surfaceProps);

    gp_Pnt centerOfMass = volumeProps.CentreOfMass();

    // Bounding Box and Sphere
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    Standard_Real xMin, yMin, zMin, xMax, yMax, zMax;
    box.Get(xMin, yMin, zMin, xMax, yMax, zMax);

    gp_Pnt boundingSphereCenter;
    Standard_Real boundingSphereRadius;
    CalculateBoundingSphere(box, boundingSphereCenter, boundingSphereRadius);

    // Process Faces
    cout << "\nFaces Information:" << endl;
    for (int i = 1; i <= faces.Extent(); ++i) {
        TopoDS_Face face = TopoDS::Face(faces(i));
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        string surfaceType = "Complex";

        if (!surface.IsNull()) {
            if (surface->DynamicType() == STANDARD_TYPE(Geom_Plane)) surfaceType = "Plane";
            else if (surface->DynamicType() == STANDARD_TYPE(Geom_CylindricalSurface)) surfaceType = "Cylinder";
        }

        cout << "    Face " << i << ": Surface Type = " << surfaceType << endl;

        // Triangulation
        BRepMesh_IncrementalMesh(face, 0.1);
        TopLoc_Location loc;
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, loc);
        if (!triangulation.IsNull()) {
            cout << "        Triangulation: " << triangulation->NbNodes() << " nodes, "
                 << triangulation->NbTriangles() << " triangles" << endl;
        }
    }

    // Final Summary
    cout << "\n========== Final Summary ==========" << endl;
    cout << "Total Number of Vertices: " << vertices.Extent() << endl;
    cout << "Total Number of Edges: " << edges.Extent() << endl;
    cout << "Total Number of Faces: " << faces.Extent() << endl;
    cout << "Volume: " << volumeProps.Mass() << " cubic units" << endl;
    cout << "Surface Area: " << surfaceProps.Mass() << " square units" << endl;
    cout << "Center of Mass: (" << centerOfMass.X() << ", " << centerOfMass.Y() << ", " << centerOfMass.Z() << ")" << endl;
    cout << "Bounding Box Min: (" << xMin << ", " << yMin << ", " << zMin << ")" << endl;
    cout << "Bounding Box Max: (" << xMax << ", " << yMax << ", " << zMax << ")" << endl;
    cout << "Bounding Sphere Center: (" << boundingSphereCenter.X() << ", " 
         << boundingSphereCenter.Y() << ", " << boundingSphereCenter.Z() << ")" << endl;
    cout << "Bounding Sphere Radius: " << boundingSphereRadius << " units" << endl;
    cout << "===================================" << endl;

    cout << "\nFeature extraction completed." << endl;
    return 0;
}
