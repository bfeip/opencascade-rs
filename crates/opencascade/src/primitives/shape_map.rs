use std::marker::PhantomData;

use cxx::UniquePtr;
use opencascade_sys::ffi;

use crate::primitives::{Edge, Face, Shape, Vertex};

/// A shape's sub-shapes of one kind, each once, numbered from 0 in the order
/// OCCT first meets them exploring the shape (`TopExp::MapShapes`).
pub struct ShapeMap<T: SubShape> {
    map: UniquePtr<ffi::TopTools_IndexedMapOfShape>,
    kind: PhantomData<T>,
}

impl<T: SubShape> ShapeMap<T> {
    pub fn new(shape: &Shape) -> Self {
        let mut map = ffi::new_indexed_map_of_shape();
        ffi::map_shapes(&shape.inner, T::TYPE, map.pin_mut());
        Self { map, kind: PhantomData }
    }

    pub fn len(&self) -> usize {
        self.map.Extent() as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The sub-shape numbered `index`.
    pub fn get(&self, index: usize) -> Option<T> {
        (index < self.len()).then(|| self.key(index))
    }

    /// The number of `sub_shape`, or `None` if it is no sub-shape of the shape.
    pub fn index_of(&self, sub_shape: &T) -> Option<usize> {
        (self.map.FindIndex(sub_shape.as_shape()) as usize).checked_sub(1)
    }

    /// The sub-shapes, in order.
    pub fn iter(&self) -> impl Iterator<Item = T> + '_ {
        (0..self.len()).map(|index| self.key(index))
    }

    fn key(&self, index: usize) -> T {
        T::from_shape(self.map.FindKey(index as i32 + 1))
    }
}

/// Which of a shape's sub-shapes of kind `A` each of its sub-shapes of kind `T`
/// lies in: the faces each edge bounds, say, or the edges that meet at each
/// vertex (`TopExp::MapShapesAndUniqueAncestors`).
///
/// OCCT topology only points downward: a face lists its edges, but an edge
/// knows nothing of the faces it bounds. Looking upward means exploring the
/// whole shape, so a map built once answers it for every sub-shape;
/// [`Shape::adjacent_faces`] is the one-off form.
///
/// Sub-shapes are numbered from 0 in an order of the map's own, which need not
/// match a [`ShapeMap`]'s. Each lists the sub-shapes it lies in once, even one
/// it occurs in twice, as a seam does in its face; one that lies in none, such
/// as a free edge among faces, lists none.
///
///
/// ```
/// // The faces next to one of a box's faces: down to its edges, then up to the
/// // face across each.
/// use opencascade::primitives::{AncestorMap, Edge, Face, Shape};
///
/// let cube = Shape::cube(1.0);
/// let edge_faces = AncestorMap::<Edge, Face>::new(&cube);
///
/// let face = cube.faces().next().unwrap();
/// let neighbours: Vec<Face> = face
///     .edges()
///     .filter_map(|edge| edge_faces.index_of(&edge))
///     .flat_map(|index| edge_faces.ancestors(index).unwrap())
///     .filter(|other| !other.is_same(&face))
///     .collect();
/// assert_eq!(neighbours.len(), 4);
/// ```
pub struct AncestorMap<T: SubShape, A: SubShape> {
    map: UniquePtr<ffi::TopTools_IndexedDataMapOfShapeListOfShape>,
    kinds: PhantomData<(T, A)>,
}

impl<T: SubShape, A: SubShape> AncestorMap<T, A> {
    pub fn new(shape: &Shape) -> Self {
        let mut map = ffi::new_indexed_data_map_of_shape_list_of_shape();
        ffi::map_shapes_and_unique_ancestors(&shape.inner, T::TYPE, A::TYPE, map.pin_mut());
        Self { map, kinds: PhantomData }
    }

    pub fn len(&self) -> usize {
        self.map.Extent() as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// The sub-shape numbered `index`.
    pub fn get(&self, index: usize) -> Option<T> {
        (index < self.len()).then(|| T::from_shape(self.map.FindKey(index as i32 + 1)))
    }

    /// The number of `sub_shape`, or `None` if it is no sub-shape of the shape.
    pub fn index_of(&self, sub_shape: &T) -> Option<usize> {
        (self.map.FindIndex(sub_shape.as_shape()) as usize).checked_sub(1)
    }

    /// The sub-shapes the one numbered `index` lies in, each once.
    pub fn ancestors(&self, index: usize) -> Option<Vec<A>> {
        let list = (index < self.len()).then(|| self.map.FindFromIndex(index as i32 + 1))?;
        Some(ffi::shape_list_to_vector(list).iter().map(A::from_shape).collect())
    }
}

/// A kind of sub-shape: a [`Face`], [`Edge`] or [`Vertex`].
pub trait SubShape: sealed::SubShape {}

impl SubShape for Face {}
impl SubShape for Edge {}
impl SubShape for Vertex {}

mod sealed {
    use opencascade_sys::ffi;

    use crate::primitives::{Edge, Face, Vertex};

    pub trait SubShape: Sized {
        const TYPE: ffi::TopAbs_ShapeEnum;

        fn from_shape(shape: &ffi::TopoDS_Shape) -> Self;

        fn as_shape(&self) -> &ffi::TopoDS_Shape;
    }

    impl SubShape for Face {
        const TYPE: ffi::TopAbs_ShapeEnum = ffi::TopAbs_ShapeEnum::TopAbs_FACE;

        fn from_shape(shape: &ffi::TopoDS_Shape) -> Self {
            Face::from_face(ffi::TopoDS_cast_to_face(shape))
        }

        fn as_shape(&self) -> &ffi::TopoDS_Shape {
            ffi::cast_face_to_shape(&self.inner)
        }
    }

    impl SubShape for Edge {
        const TYPE: ffi::TopAbs_ShapeEnum = ffi::TopAbs_ShapeEnum::TopAbs_EDGE;

        fn from_shape(shape: &ffi::TopoDS_Shape) -> Self {
            Edge::from_edge(ffi::TopoDS_cast_to_edge(shape))
        }

        fn as_shape(&self) -> &ffi::TopoDS_Shape {
            ffi::cast_edge_to_shape(&self.inner)
        }
    }

    impl SubShape for Vertex {
        const TYPE: ffi::TopAbs_ShapeEnum = ffi::TopAbs_ShapeEnum::TopAbs_VERTEX;

        fn from_shape(shape: &ffi::TopoDS_Shape) -> Self {
            Vertex::from_vertex(ffi::TopoDS_cast_to_vertex(shape))
        }

        fn as_shape(&self) -> &ffi::TopoDS_Shape {
            ffi::cast_vertex_to_shape(&self.inner)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_maps_each_sub_shape_once() {
        let cube = Shape::cube(1.0);
        let (faces, edges, vertices) =
            (ShapeMap::<Face>::new(&cube), ShapeMap::<Edge>::new(&cube), ShapeMap::<Vertex>::new(&cube));
        assert_eq!((faces.len(), edges.len(), vertices.len()), (6, 12, 8));

        for (index, face) in faces.iter().enumerate() {
            assert_eq!(faces.index_of(&face), Some(index));
        }
        // Each edge is met once per face it bounds, and numbered once.
        for edge in cube.edges() {
            let index = edges.index_of(&edge).expect("every box edge is mapped");
            assert!(edges.get(index).unwrap().is_same(&edge));
        }
        assert!(faces.get(6).is_none());
    }

    #[test]
    fn a_box_edge_lies_in_two_faces() {
        let cube = Shape::cube(1.0);
        let edges = AncestorMap::<Edge, Face>::new(&cube);
        assert_eq!(edges.len(), 12);
        for index in 0..edges.len() {
            let edge = edges.get(index).unwrap();
            assert_eq!(edges.index_of(&edge), Some(index));
            let faces = edges.ancestors(index).unwrap();
            assert_eq!(faces.len(), 2);
            assert!(faces.iter().all(|face| face.edges().any(|bounding| bounding.is_same(&edge))));
        }
        assert!(edges.ancestors(12).is_none());
    }

    /// A free edge listed before a face: the ancestor map numbers the face's
    /// edges first, and the free edge lies in no face.
    #[test]
    fn an_ancestor_map_numbers_in_its_own_order() {
        let free = Edge::segment(glam::dvec3(5.0, 0.0, 0.0), glam::dvec3(6.0, 0.0, 0.0)).unwrap();
        let square = crate::primitives::Wire::rect(1.0, 1.0).unwrap();
        let face = Face::from_wire(&square).unwrap();
        let shape = Shape::from(crate::primitives::Compound::from_shapes([Shape::from(free), Shape::from(face)]));
        let free = shape.edges().next().unwrap();

        let (edges, edge_faces) = (ShapeMap::<Edge>::new(&shape), AncestorMap::<Edge, Face>::new(&shape));
        assert_eq!((edges.len(), edge_faces.len()), (5, 5));
        assert_eq!(edges.index_of(&free), Some(0));
        let index = edge_faces.index_of(&free).expect("the free edge is mapped");
        assert_eq!(index, 4, "the free edge comes after the face's");
        assert!(edge_faces.ancestors(index).unwrap().is_empty());
    }

    /// A cylinder's seam occurs twice in its side face, which it lists once.
    #[test]
    fn a_seam_lists_its_face_once() {
        let cylinder = Shape::cylinder_radius_height(1.0, 2.0);
        let seam = cylinder.seam_edges().into_iter().next().expect("a cylinder has a seam");
        let edge_faces = AncestorMap::<Edge, Face>::new(&cylinder);
        let index = edge_faces.index_of(&seam).unwrap();
        assert_eq!(edge_faces.ancestors(index).unwrap().len(), 1);
    }

    #[test]
    fn a_foreign_sub_shape_has_no_index() {
        let edges = ShapeMap::<Edge>::new(&Shape::cube(1.0));
        let elsewhere = Edge::segment(glam::DVec3::ZERO, glam::DVec3::X).unwrap();
        assert_eq!(edges.index_of(&elsewhere), None);
    }
}
