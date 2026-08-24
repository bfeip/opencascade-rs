use glam::DVec3;
use opencascade::{
    angle::{RVec, ToAngle},
    primitives::{IntoShape, Shape, Shell, Wire},
    workplane::Workplane,
    Error,
};

pub fn shape() -> Result<Shape, Error> {
    let r = 10.0;
    let a = 5.0;

    let wire_profile: Wire = Workplane::xz()
        .rotated(RVec::z(45.0.degrees()))
        .translated(DVec3::new(-r, 0.0, 0.0))
        .rect(a, a);

    let path: Wire = Workplane::xy().sketch().arc((-r, 0.0), (0.0, r), (r, 0.0)).wire();

    let pipe_shell: Shell = wire_profile.sweep_along(&path);

    Ok(pipe_shell.into_shape())
}
