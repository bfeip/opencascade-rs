use crate::{
    primitives::{FaceOrientation, Shape},
    Error,
};
use cxx::UniquePtr;
use glam::{dvec2, dvec3, DVec2, DVec3};
use opencascade_sys::ffi;

/// Triangle index range for a single B-Rep face within a tessellated mesh.
///
/// `start` and `count` are in triangles (not raw indices). Multiply by 3 to
/// get the raw index offset into `Mesh::indices`.
#[derive(Debug, Clone, Copy)]
pub struct FaceRange {
    /// First triangle index belonging to this face.
    pub start: u32,
    /// Number of triangles belonging to this face.
    pub count: u32,
}

#[derive(Debug)]
pub struct Mesh {
    pub vertices: Vec<DVec3>,
    pub uvs: Vec<DVec2>,
    pub normals: Vec<DVec3>,
    pub indices: Vec<usize>,
}

pub struct Mesher {
    pub(crate) inner: UniquePtr<ffi::BRepMesh_IncrementalMesh>,
}

impl Mesher {
    pub fn try_new(shape: &Shape, triangulation_tolerance: f64) -> Result<Self, Error> {
        let inner = ffi::BRepMesh_IncrementalMesh_ctor(&shape.inner, triangulation_tolerance);

        if inner.IsDone() {
            Ok(Self { inner })
        } else {
            Err(Error::TriangulationFailed)
        }
    }

    pub fn mesh(self) -> Result<Mesh, Error> {
        let (mesh, _) = self.mesh_with_face_ranges()?;
        Ok(mesh)
    }

    /// Tessellates the shape and returns the flat mesh alongside per-face index ranges.
    ///
    /// Each [`FaceRange`] records the start triangle and count for one B-Rep face within
    /// the flat `Mesh::indices` list (units: triangles, not raw indices).
    pub fn mesh_with_face_ranges(mut self) -> Result<(Mesh, Vec<FaceRange>), Error> {
        let mut vertices = vec![];
        let mut uvs = vec![];
        let mut normals = vec![];
        let mut indices = vec![];
        let mut face_ranges: Vec<FaceRange> = vec![];

        let triangulated_shape = Shape::from_shape(self.inner.pin_mut().Shape());

        for face in triangulated_shape.faces() {
            let tri_start = (indices.len() / 3) as u32;

            let mut location = ffi::TopLoc_Location_ctor();

            let triangulation_handle =
                ffi::BRep_Tool_Triangulation(&face.inner, location.pin_mut());

            let triangulation = ffi::HandlePoly_Triangulation_Get(&triangulation_handle)
                .map_err(|_| Error::UntriangulatedFace)?;

            let index_offset = vertices.len();
            let face_point_count = triangulation.NbNodes();

            for i in 1..=face_point_count {
                let mut point = ffi::Poly_Triangulation_Node(triangulation, i);
                point.pin_mut().Transform(&ffi::TopLoc_Location_Transformation(&location));
                vertices.push(dvec3(point.X(), point.Y(), point.Z()));
            }

            let mut u_min = f64::INFINITY;
            let mut v_min = f64::INFINITY;

            let mut u_max = f64::NEG_INFINITY;
            let mut v_max = f64::NEG_INFINITY;

            for i in 1..=(face_point_count) {
                let uv = ffi::Poly_Triangulation_UV(triangulation, i);
                let (u, v) = (uv.X(), uv.Y());

                u_min = u_min.min(u);
                v_min = v_min.min(v);

                u_max = u_max.max(u);
                v_max = v_max.max(v);

                uvs.push(dvec2(u, v));
            }

            // Normalize the newly added UV coordinates.
            for uv in &mut uvs[index_offset..(index_offset + face_point_count as usize)] {
                uv.x = (uv.x - u_min) / (u_max - u_min);
                uv.y = (uv.y - v_min) / (v_max - v_min);

                if face.orientation() != FaceOrientation::Forward {
                    uv.x = 1.0 - uv.x;
                }
            }

            // Add in the normals.
            ffi::compute_normals(&face.inner, &triangulation_handle);

            // OCCT normals are oriented along the underlying surface's parametric normal,
            // regardless of face orientation. For Reversed faces the winding is already
            // flipped above so the triangle is front-facing, but the surface normal still
            // points inward. Negate those normals so they agree with the winding.
            let normal_sign = if face.orientation() == FaceOrientation::Forward { 1.0 } else { -1.0 };

            // The triangulation stores normals in the face's local coordinate space.
            // Apply the same location transform used for vertices so that normals end
            // up in the same world space. gp_Dir::Transform applies only the rotational
            // part of the transform (no translation), which is correct for directions.
            let location_is_identity = ffi::TopLoc_Location_IsIdentity(&location);
            let transform = if !location_is_identity {
                Some(ffi::TopLoc_Location_Transformation(&location))
            } else {
                None
            };

            for i in 1..=face_point_count as usize {
                let mut normal = ffi::Poly_Triangulation_Normal(triangulation, i as i32);
                if let Some(ref t) = transform {
                    normal.pin_mut().Transform(t);
                }
                normals.push(dvec3(
                    normal.X() * normal_sign,
                    normal.Y() * normal_sign,
                    normal.Z() * normal_sign,
                ));
            }

            for i in 1..=triangulation.NbTriangles() {
                let triangle = triangulation.Triangle(i);

                if face.orientation() == FaceOrientation::Forward {
                    indices.push(index_offset + triangle.Value(1) as usize - 1);
                    indices.push(index_offset + triangle.Value(2) as usize - 1);
                    indices.push(index_offset + triangle.Value(3) as usize - 1);
                } else {
                    indices.push(index_offset + triangle.Value(3) as usize - 1);
                    indices.push(index_offset + triangle.Value(2) as usize - 1);
                    indices.push(index_offset + triangle.Value(1) as usize - 1);
                }
            }

            let tri_count = (indices.len() / 3) as u32 - tri_start;
            face_ranges.push(FaceRange { start: tri_start, count: tri_count });
        }

        Ok((Mesh { vertices, uvs, normals, indices }, face_ranges))
    }
}
