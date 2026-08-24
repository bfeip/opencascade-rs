use glam::DVec3;
use opencascade::{
    primitives::{IntoShape, JoinType, Shape},
    workplane::Workplane,
    Error,
};

// Demonstrates ofsetting a face or wire in 2D.

pub fn shape() -> Result<Shape, Error> {
    let mut shapes = vec![];

    for (i, join_type) in [JoinType::Arc, JoinType::Intersection].into_iter().enumerate() {
        let solid = Workplane::xy()
            .translated(DVec3::new(32.0 * i as f64, 0.0, 0.0))
            .sketch()
            .move_to(0.0, 10.0)
            .line_to(5.0, 5.0)
            .line_to(5.0, -5.0)
            .line_to(0.0, 0.0)
            .line_to(-5.0, -5.0)
            .line_to(-5.0, 5.0)
            .close()
            .offset(1.0, join_type) // offset a wire
            .to_face()?
            .extrude(DVec3::new(0.0, 0.0, 8.0))
            .into_shape();

        shapes.push(solid);
    }

    let mut shapes = shapes.into_iter();
    let mut result = shapes.next().expect("we always build at least one shape");

    for shape in shapes {
        result = result.union(&shape)?.shape;
    }

    Ok(result)
}
